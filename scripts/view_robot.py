#!/usr/bin/env python3
"""
view_robot.py — 基于胶囊体的串联机器人 MuJoCo 可视化工具

读取机器人 config/kinematic_params.yaml 中的运动学参数，自动构建胶囊体几何模型，
在 MuJoCo 视窗中显示。渲染线程与动力学线程分离（参考 revoarm_simulator.py）。

Usage:
    conda activate mj_revoarm
    python3 scripts/view_robot.py single_right
    python3 scripts/view_robot.py revoarm_right
    python3 scripts/view_robot.py revoarm_left
"""

import os
import sys
import random
import threading
import time

import numpy as np
import yaml
import mujoco
import mujoco.viewer


# ===========================================================================
# YAML 加载
# ===========================================================================

def _resolve_robot_dir(robot_name: str) -> str:
    """解析机器人目录的绝对路径。"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    robot_dir = os.path.join(project_root, "robots", robot_name)
    if not os.path.isdir(robot_dir):
        raise FileNotFoundError(f"机器人目录不存在: {robot_dir}")
    return robot_dir


def load_kinematic_params(robot_name: str) -> dict:
    """读取 robots/{robot_name}/config/kinematic_params.yaml 并返回解析后的字典。"""
    robot_dir = _resolve_robot_dir(robot_name)
    yaml_path = os.path.join(robot_dir, "config", "kinematic_params.yaml")
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"运动学参数文件不存在: {yaml_path}")

    with open(yaml_path, "r") as f:
        params = yaml.safe_load(f)

    if "bodies" not in params:
        raise ValueError(f"{yaml_path} 中缺少 'bodies' 字段")

    return params


# ===========================================================================
# 辅助数学函数
# ===========================================================================

def _normalize(v: np.ndarray) -> np.ndarray:
    """归一化向量，零向量返回自身。"""
    norm = np.linalg.norm(v)
    if norm < 1e-12:
        return v
    return v / norm



# ===========================================================================
# Body 数据预处理
# ===========================================================================

def _ensure_defaults(body: dict) -> dict:
    """为 body 填充缺失的默认字段。"""
    body.setdefault("pos", [0.0, 0.0, 0.0])
    body.setdefault("quat", [1.0, 0.0, 0.0, 0.0])
    body.setdefault("mass", 0.0)
    body.setdefault("joint_axis", [0.0, 0.0, 1.0])
    body.setdefault("has_joint", False)
    return body


# ===========================================================================
# 胶囊体参数计算
# ===========================================================================

def _compute_capsule_for_body(bodies: list, i: int) -> dict | None:
    """
    为第 i 个 body 计算胶囊体的 fromto 字符串和半径。

    胶囊体表示从本 body 关节到下一 body 关节之间的连杆段。
    对于最后一个 body，使用关节轴方向 + 默认长度。

    返回: {"fromto": str, "radius": float} 或 None
    """
    body = bodies[i]
    if not body.get("has_joint"):
        return None

    joint_axis = np.array(body.get("joint_axis", [0.0, 0.0, 1.0]), dtype=float)

    # ---- 确定连杆向量（从本关节到下一关节） ----
    if i + 1 < len(bodies):
        # 下一 body 的 pos 即为从本 body 到下一 body 的偏移
        next_pos = np.array(bodies[i + 1].get("pos", [0.0, 0.0, 0.0]), dtype=float)
        link_vector = next_pos
    else:
        link_vector = np.zeros(3)

    link_length = float(np.linalg.norm(link_vector))

    # ---- 零长度处理：尝试使用本 body 的 pos（从父关节到本关节） ----
    if link_length < 0.005:
        own_pos = np.array(body.get("pos", [0.0, 0.0, 0.0]), dtype=float)
        own_length = float(np.linalg.norm(own_pos))
        if own_length >= 0.005:
            link_vector = own_pos
            link_length = own_length

    # ---- 仍为零：使用关节轴方向 + 默认长度 ----
    if link_length < 0.005:
        link_vector = _normalize(joint_axis) * 0.06
        link_length = 0.06

    # ---- 固定胶囊体半径 ----
    radius = 0.03  # 固定 3cm 半径，不受质量/长度影响

    # ---- 构建 fromto 字符串 ----
    # 在本 body 坐标系中：from=原点(关节), to=link_vector 终点(下一关节)
    fromto = (f"{0.0:.6f} {0.0:.6f} {0.0:.6f}  "
              f"{link_vector[0]:.6f} {link_vector[1]:.6f} {link_vector[2]:.6f}")

    return {"fromto": fromto, "radius": radius}


# ===========================================================================
# MJCF XML 构建
# ===========================================================================

def _build_chain_xml(bodies: list, idx: int, indent: int) -> str:
    """递归构建串联机械臂的 MJCF body 链。

    串联机械臂中 bodies 按父子顺序排列：bodies[0] 为根，
    bodies[i+1] 是 bodies[i] 的子 body。
    """
    if idx >= len(bodies):
        return ""

    prefix = "  " * indent
    body = bodies[idx]
    name = body["name"]
    pos = body.get("pos", [0.0, 0.0, 0.0])
    quat = body.get("quat", [1.0, 0.0, 0.0, 0.0])
    has_joint = body.get("has_joint", False)
    joint_axis = body.get("joint_axis", [0.0, 0.0, 1.0])

    pos_str = f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}"
    quat_str = f"{quat[0]:.8f} {quat[1]:.8f} {quat[2]:.8f} {quat[3]:.8f}"

    if has_joint:
        # 随机色相（HSV→RGB），饱和度 0.7，明度 0.85，保证颜色鲜明但不刺眼
        h = random.random()
        s, v = 0.7, 0.85
        # HSV → RGB
        i_idx = int(h * 6)
        f = h * 6 - i_idx
        p = v * (1 - s)
        q = v * (1 - f * s)
        t = v * (1 - (1 - f) * s)
        rgb = [
            (v, t, p), (q, v, p), (p, v, t),
            (p, q, v), (t, p, v), (v, p, q),
        ][i_idx % 6]
        rgba = f"{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1.0"
    else:
        rgba = "0.55 0.55 0.55 1.0"   # 固定底座：灰色

    lines = [f'{prefix}<body name="{name}" pos="{pos_str}" quat="{quat_str}">']

    # 关节
    if has_joint:
        axis_str = f"{joint_axis[0]:.6f} {joint_axis[1]:.6f} {joint_axis[2]:.6f}"
        lines.append(f'{prefix}  <joint name="{name}_joint" type="hinge" axis="{axis_str}"'
                     f' limited="true" range="-3.14 3.14"/>')

    # 胶囊体 / 方块 geom
    capsule = _compute_capsule_for_body(bodies, idx)
    if capsule is not None:
        lines.append(f'{prefix}  <geom type="capsule" fromto="{capsule["fromto"]}"'
                     f' size="{capsule["radius"]:.6f}" rgba="{rgba}"/>')
    else:
        lines.append(f'{prefix}  <geom type="box" size="0.04 0.04 0.04" rgba="{rgba}"/>')

    # 递归子 body
    child_xml = _build_chain_xml(bodies, idx + 1, indent + 1)
    if child_xml:
        lines.append(child_xml)

    lines.append(f"{prefix}</body>")
    return "\n".join(lines)


def build_mjcf_xml(params: dict, robot_name: str) -> str:
    """基于运动学参数构建完整的 MJCF XML 字符串。"""
    bodies_raw = params.get("bodies", [])
    bodies = [_ensure_defaults(b.copy()) for b in bodies_raw]

    # 构建 worldbody 内的 body 链（串联机械臂：顺序父子关系）
    body_xml = _build_chain_xml(bodies, idx=0, indent=2)

    xml = f"""<mujoco model="{robot_name}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" gravity="0 0 0"/>
  <default>
    <geom contype="0" conaffinity="0" condim="3"/>
    <joint limited="true" range="-3.14 3.14" damping="0.5"/>
  </default>
  <worldbody>
    <light name="light1" pos="2 2 3" dir="-0.5 -0.5 -1" diffuse="0.8 0.8 0.8"/>
    <light name="light2" pos="-1 -1 2" dir="0.5 0.5 -1" diffuse="0.4 0.4 0.4"/>
    <geom name="floor" type="plane" pos="0 0 -1.2" size="3 3 0.1" rgba="0.85 0.85 0.85 1.0"/>
{body_xml}
  </worldbody>
</mujoco>"""

    return xml


# ===========================================================================
# MuJoCo 视窗管理器
# ===========================================================================

class RobotViewer:
    """管理 MuJoCo 模型、数据和被动视窗，分离渲染/动力学线程。

    线程架构：
      - 线程 A（动力学, ~1000 Hz）: mj_step() 推进仿真
      - 线程 B（渲染,   ~60 Hz）:  viewer.sync() 刷新画面

    所有 mj_data 的访问由 threading.Lock() 保护。
    """

    def __init__(self, xml_string: str):
        self.mj_model = mujoco.MjModel.from_xml_string(xml_string)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.lock = threading.Lock()
        self.dt = self.mj_model.opt.timestep
        self._running = True

        print(f"模型加载完成: DOF={self.mj_model.nv}, "
              f"timestep={self.dt:.4f}s ({1.0/self.dt:.0f} Hz)")

        # 启动被动视窗
        self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        time.sleep(0.3)  # 等待视窗初始化

        print("视窗已启动。按关闭按钮或 Ctrl+C 退出。")

    # ------------------------------------------------------------------
    def _dynamics_thread(self) -> None:
        """线程 A: 物理仿真步进，按模型时间步长运行。"""
        while self._running and self.viewer.is_running():
            step_start = time.perf_counter()

            with self.lock:
                mujoco.mj_step(self.mj_model, self.mj_data)

            # 保持恒定步进速率
            elapsed = time.perf_counter() - step_start
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------
    def _viewer_thread(self) -> None:
        """线程 B: 视窗渲染刷新，~60 Hz。"""
        while self._running and self.viewer.is_running():
            with self.lock:
                self.viewer.sync()
            time.sleep(0.016)  # ~60 FPS

    # ------------------------------------------------------------------
    def run(self) -> None:
        """启动所有线程并阻塞直到用户关闭视窗。"""
        threads = [
            threading.Thread(target=self._dynamics_thread, daemon=True, name="dynamics"),
            threading.Thread(target=self._viewer_thread, daemon=True, name="viewer"),
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
        print(f"\n可用的机器人:")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        robots_dir = os.path.join(os.path.dirname(script_dir), "robots")
        if os.path.isdir(robots_dir):
            for name in sorted(os.listdir(robots_dir)):
                yaml_path = os.path.join(robots_dir, name, "config", "kinematic_params.yaml")
                if os.path.isfile(yaml_path):
                    print(f"  - {name}")
        sys.exit(1)

    robot_name = sys.argv[1]

    # 1. 加载运动学参数
    print(f"加载机器人 '{robot_name}' 的运动学参数...")
    params = load_kinematic_params(robot_name)
    print(f"  robot_type: {params.get('robot_type', 'unknown')}")
    print(f"  dof: {params.get('dof', '?')}")
    print(f"  bodies: {len(params.get('bodies', []))}")

    # 2. 构建 MJCF XML
    print("构建 MuJoCo 模型...")
    xml_string = build_mjcf_xml(params, robot_name)

    # 3. 启动视窗
    viewer = RobotViewer(xml_string)
    viewer.run()


if __name__ == "__main__":
    main()
