#!/usr/bin/env python3
"""
view_traj.py — 机器人激励轨迹循环回放可视化

基于 view_robot.py，在显示胶囊体机器人模型的基础上，
循环回放 result_inertia/*_excitation_trajectory.csv 中的激励轨迹。

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
    build_mjcf_xml,
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
    result_dir = os.path.join(robot_dir, "result_inertia")

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
# 轨迹回放视窗
# ===========================================================================

class TrajectoryViewer:
    """管理 MuJoCo 模型、轨迹回放和视窗。

    线程架构：
      - 线程 A（渲染,    ~60 Hz）: viewer.sync() 刷新画面
      - 线程 B（轨迹控制, ~CSV 帧率）: 直接设置 mj_data.qpos + mj_forward()

    不通过 PD 控制器驱动，直接写入关节状态实现平滑播放。
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

        print("视窗已启动。按关闭按钮或 Ctrl+C 退出。")

    # ------------------------------------------------------------------
    def _viewer_thread(self) -> None:
        """线程 A: 视窗渲染刷新。"""
        while self._running and self.viewer.is_running():
            with self.lock:
                self.viewer.sync()
            time.sleep(0.016)

    # ------------------------------------------------------------------
    def _trajectory_thread(self) -> None:
        """线程 B: 按 CSV 帧率循环更新关节状态。

        直接写入 mj_data.qpos 并调用 mj_forward() 更新运动学，
        绕过 PD 控制器，实现无抖动的平滑播放。

        到达轨迹末尾时不直接跳回第 0 帧，而是通过线性插值平滑过渡
        （过渡时长 = 1.0s），避免位置跳变。
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
                    self.mj_data.qpos[:] = q_target
                    mujoco.mj_forward(self.mj_model, self.mj_data)

                self.traj_idx += 1
                # 过渡完成，回到第 0 帧
                if self.traj_idx >= self.n_frames:
                    self.traj_idx = 0
            else:
                # ---- 正常回放 ----
                q_target = self.q_traj[self.traj_idx]

                with self.lock:
                    self.mj_data.qpos[:] = q_target
                    mujoco.mj_forward(self.mj_model, self.mj_data)

                self.traj_idx += 1

            elapsed = time.perf_counter() - loop_start
            sleep_time = self.dt_csv - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------
    def run(self) -> None:
        """启动所有线程并阻塞直到用户关闭视窗。"""
        threads = [
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

    # 3. 构建 MJCF XML（无执行器，直接设置关节状态播放）
    print("构建 MuJoCo 模型...")
    xml_string = build_mjcf_xml(params, robot_name)

    # 4. 启动视窗 + 轨迹回放
    viewer = TrajectoryViewer(xml_string, q_traj, dt_csv)
    viewer.run()


if __name__ == "__main__":
    main()
