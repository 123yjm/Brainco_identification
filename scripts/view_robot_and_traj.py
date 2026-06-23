#!/usr/bin/env python3
"""
view_traj.py — 机器人激励轨迹循环回放可视化

基于 view_robot.py，在显示胶囊体机器人模型的基础上，
循环回放 result/*_excitation_trajectory.csv 中的激励轨迹。

Usage:
    conda activate mj_revoarm
    python3 scripts/view_traj.py single_right
"""

import os
import sys
import threading
import time

import numpy as np
import mujoco
import mujoco.viewer

# 复用 view_robot.py 的模块
from view_robot import (
    load_kinematic_params,
    _resolve_robot_dir,
    _ensure_defaults,
    _compute_capsule_for_body,
    _build_chain_xml,
)


# ===========================================================================
# 轨迹 CSV 加载
# ===========================================================================

def load_trajectory_csv(robot_name: str) -> tuple:
    """读取激励轨迹 CSV，返回 (q_array, dt)。

    q_array: shape (N, 7)，7 个关节的目标位置 (rad)
    dt: CSV 采样间隔 (s)
    """
    robot_dir = _resolve_robot_dir(robot_name)
    result_dir = os.path.join(robot_dir, "result")

    # 查找 *_excitation_trajectory.csv
    csv_path = None
    for fname in os.listdir(result_dir):
        if fname.endswith("_excitation_trajectory.csv"):
            csv_path = os.path.join(result_dir, fname)
            break

    if csv_path is None:
        raise FileNotFoundError(
            f"在 {result_dir} 中未找到 *_excitation_trajectory.csv"
        )

    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    t = data[:, 0]
    q = data[:, 1:8]  # 列 1-7: q0..q6

    dt = float(np.mean(np.diff(t)))
    print(f"轨迹加载: {csv_path}")
    print(f"  采样点: {len(t)}, dt={dt:.4f}s, 时长={t[-1]:.1f}s")
    print(f"  q 范围: [{q.min():.3f}, {q.max():.3f}] rad")

    return q, dt


# ===========================================================================
# 带执行器的 MJCF XML 构建
# ===========================================================================

def _collect_joint_bodies(bodies: list) -> list:
    """收集所有带关节的 body 名称。"""
    joint_names = []
    for b in bodies:
        if b.get("has_joint"):
            joint_names.append(b["name"])
    return joint_names


def build_mjcf_xml_with_actuators(params: dict, robot_name: str) -> str:
    """构建带 position 执行器的 MJCF XML 字符串。"""
    bodies_raw = params.get("bodies", [])
    bodies = [_ensure_defaults(b.copy()) for b in bodies_raw]
    joint_names = _collect_joint_bodies(bodies)

    # body 链 XML（与 view_robot 相同）
    body_xml = _build_chain_xml(bodies, idx=0, indent=2)

    # actuator 区域
    actuator_lines = []
    for jn in joint_names:
        actuator_lines.append(
            f'    <position name="act_{jn}" joint="{jn}_joint"'
            f' kp="100" kv="20" forcerange="-50 50"/>'
        )
    actuator_xml = "\n".join(actuator_lines)

    xml = f"""<mujoco model="{robot_name}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" gravity="0 0 0"/>
  <default>
    <geom contype="0" conaffinity="0" condim="3"/>
    <joint limited="true" range="-3.14 3.14"/>
    <position kp="200"/>
  </default>
  <worldbody>
    <light name="light1" pos="2 2 3" dir="-0.5 -0.5 -1" diffuse="0.8 0.8 0.8"/>
    <light name="light2" pos="-1 -1 2" dir="0.5 0.5 -1" diffuse="0.4 0.4 0.4"/>
    <geom name="floor" type="plane" pos="0 0 -1.2" size="3 3 0.1" rgba="0.85 0.85 0.85 1.0"/>
{body_xml}
  </worldbody>
  <actuator>
{actuator_xml}
  </actuator>
</mujoco>"""

    return xml


# ===========================================================================
# 轨迹回放视窗
# ===========================================================================

class TrajectoryViewer:
    """管理 MuJoCo 模型、轨迹回放和视窗。

    线程架构：
      - 线程 A（动力学, ~1000 Hz）: mj_step() 推进仿真
      - 线程 B（渲染,    ~60 Hz）: viewer.sync() 刷新画面
      - 线程 C（轨迹控制, ~100 Hz）: 按 CSV 帧率更新 mj_data.ctrl

    所有 mj_data 的访问由 threading.Lock() 保护。
    """

    def __init__(self, xml_string: str, q_traj: np.ndarray, dt_csv: float):
        self.mj_model = mujoco.MjModel.from_xml_string(xml_string)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.lock = threading.Lock()
        self.dt = self.mj_model.opt.timestep
        self._running = True

        self.q_traj = q_traj
        self.dt_csv = dt_csv
        self.traj_idx = 0
        self.n_frames = len(q_traj)

        print(f"模型加载完成: DOF={self.mj_model.nv}, nu={self.mj_model.nu}, "
              f"timestep={self.dt:.4f}s ({1.0/self.dt:.0f} Hz)")
        print(f"轨迹帧数: {self.n_frames}, 帧率: {1.0/dt_csv:.0f} Hz")

        # 初始位置设为轨迹第一帧
        self.mj_data.qpos[:] = q_traj[0]
        mujoco.mj_forward(self.mj_model, self.mj_data)

        # 启动被动视窗
        self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        time.sleep(0.3)

        # 首帧 ctrl 也设为轨迹第一帧，避免从 0 跳变
        self.mj_data.ctrl[:] = q_traj[0]

        print("视窗已启动。按关闭按钮或 Ctrl+C 退出。")

    # ------------------------------------------------------------------
    def _dynamics_thread(self) -> None:
        """线程 A: 物理仿真步进。"""
        while self._running and self.viewer.is_running():
            step_start = time.perf_counter()

            with self.lock:
                mujoco.mj_step(self.mj_model, self.mj_data)

            elapsed = time.perf_counter() - step_start
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------
    def _viewer_thread(self) -> None:
        """线程 B: 视窗渲染刷新。"""
        while self._running and self.viewer.is_running():
            with self.lock:
                self.viewer.sync()
            time.sleep(0.016)

    # ------------------------------------------------------------------
    def _trajectory_thread(self) -> None:
        """线程 C: 按 CSV 帧率循环更新目标位置。

        到达轨迹末尾时不直接跳回第 0 帧，而是通过线性插值平滑过渡
        （过渡时长 = 1.0s），避免位置跳变导致的剧烈抖动。
        """
        TRANSITION_TIME = 1.0       # 循环过渡时长 (s)
        n_transition = max(1, int(TRANSITION_TIME / self.dt_csv))

        while self._running and self.viewer.is_running():
            loop_start = time.perf_counter()

            # 检查是否需要平滑过渡（接近末尾）
            remaining = self.n_frames - self.traj_idx

            if remaining <= n_transition:
                # ---- 平滑过渡：从当前帧线性插值到第 0 帧 ----
                q_from = self.q_traj[self.traj_idx]                    # 当前帧
                q_to = self.q_traj[0]                                   # 第 0 帧
                t = float(n_transition - remaining) / float(n_transition)  # 0→1
                q_target = q_from + (q_to - q_from) * t

                with self.lock:
                    self.mj_data.ctrl[:] = q_target

                self.traj_idx += 1
                # 过渡完成，回到第 0 帧
                if self.traj_idx >= self.n_frames:
                    self.traj_idx = 0
            else:
                # ---- 正常回放 ----
                q_target = self.q_traj[self.traj_idx]

                with self.lock:
                    self.mj_data.ctrl[:] = q_target

                self.traj_idx += 1

            elapsed = time.perf_counter() - loop_start
            sleep_time = self.dt_csv - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------
    def run(self) -> None:
        """启动所有线程并阻塞直到用户关闭视窗。"""
        threads = [
            threading.Thread(target=self._dynamics_thread, daemon=True, name="dynamics"),
            threading.Thread(target=self._viewer_thread, daemon=True, name="viewer"),
            threading.Thread(target=self._trajectory_thread, daemon=True, name="trajectory"),
        ]

        for t in threads:
            t.start()

        try:
            while self.viewer.is_running():
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n接收到中断信号，正在退出...")
        finally:
            self._running = False
            for t in threads:
                t.join(timeout=2.0)
            print("视窗已关闭。")


# ===========================================================================
# 入口
# ===========================================================================

def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {os.path.basename(__file__)} <robot_name>")
        print(f"Example: python3 {os.path.basename(__file__)} single_right")
        sys.exit(1)

    robot_name = sys.argv[1]

    # 1. 加载运动学参数
    print(f"加载机器人 '{robot_name}' 的运动学参数...")
    params = load_kinematic_params(robot_name)
    print(f"  robot_type: {params.get('robot_type', 'unknown')}")
    print(f"  dof: {params.get('dof', '?')}")

    # 2. 加载轨迹
    q_traj, dt_csv = load_trajectory_csv(robot_name)

    # 3. 构建 MJCF XML（带执行器）
    print("构建 MuJoCo 模型（含 position 执行器）...")
    xml_string = build_mjcf_xml_with_actuators(params, robot_name)

    # 4. 启动视窗 + 轨迹回放
    viewer = TrajectoryViewer(xml_string, q_traj, dt_csv)
    viewer.run()


if __name__ == "__main__":
    main()
