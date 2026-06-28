#!/usr/bin/env python3
"""
将 PC 辨识得到的惯性参数（body-origin 坐标系）转换为 URDF COM 坐标系，
并回填到 URDF 文件的 right_arm_link1~6 + right_connector_link 中。

辨识参数格式（每 body 10 个，body-origin 坐标系）:
  [mass, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]

URDF 格式（COM 坐标系）:
  <origin xyz="cx cy cz"/>   （cx = mx/mass, cy = my/mass, cz = mz/mass）
  <mass value="mass"/>
  <inertia ixx="Ixx_com" .../>

并行轴定理（body-origin → COM 反向推导）:
  cx = mx/mass, cy = my/mass, cz = mz/mass
  Ixx_com = Ixx_origin - mass * (cy² + cz²)
  Iyy_com = Iyy_origin - mass * (cx² + cz²)
  Izz_com = Izz_origin - mass * (cx² + cy²)
  Ixy_com = Ixy_origin + mass * cx * cy
  Ixz_com = Ixz_origin + mass * cx * cz
  Iyz_com = Iyz_origin + mass * cy * cz

物理可行性检查:
  - mass > 0
  - Ixx, Iyy, Izz > 0（惯性矩阵正定）
  - Ixx + Iyy > Izz, Ixx + Izz > Iyy, Iyy + Izz > Ixx（三角不等式）
  若不满足则 clamp 到最小有效值并打印警告
"""

import yaml
import re
import math
import sys
import shutil

# 7 个辨识 body 到 URDF link 名称的映射
BODY_TO_LINK = [
    "right_arm_link1",
    "right_arm_link2",
    "right_arm_link3",
    "right_arm_link4",
    "right_arm_link5",
    "right_arm_link6",
    "right_connector_link",
]

# 最小正惯性（用于 clamp 负值到物理可行）
MIN_INERTIA = 1e-8
MIN_MASS = 1e-6


def body_origin_to_com(params_10):
    """
    将 body-origin 坐标系的 10 参数转换为 COM 坐标系参数。
    输入: [mass, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
    输出: dict with keys: mass, cx, cy, cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz
    """
    mass, mx, my, mz = params_10[0], params_10[1], params_10[2], params_10[3]
    Ixx_o, Ixy_o, Ixz_o, Iyy_o, Iyz_o, Izz_o = (
        params_10[4], params_10[5], params_10[6],
        params_10[7], params_10[8], params_10[9],
    )

    if mass < MIN_MASS:
        return {"mass": MIN_MASS, "cx": 0, "cy": 0, "cz": 0,
                "Ixx": MIN_INERTIA, "Iyy": MIN_INERTIA, "Izz": MIN_INERTIA,
                "Ixy": 0, "Ixz": 0, "Iyz": 0,
                "warnings": ["mass <= 0, replaced with tiny value"]}

    cx = mx / mass
    cy = my / mass
    cz = mz / mass

    # 并行轴定理反向：I_com = I_origin - mass * (d²I - d*d^T)
    Ixx = Ixx_o - mass * (cy * cy + cz * cz)
    Iyy = Iyy_o - mass * (cx * cx + cz * cz)
    Izz = Izz_o - mass * (cx * cx + cy * cy)
    Ixy = Ixy_o + mass * cx * cy
    Ixz = Ixz_o + mass * cx * cz
    Iyz = Iyz_o + mass * cy * cz

    warnings = []

    # 物理可行性检查与修正
    Ixx, Iyy, Izz, fix_msgs = clamp_physically_feasible(Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
    warnings.extend(fix_msgs)

    return {"mass": mass, "cx": cx, "cy": cy, "cz": cz,
            "Ixx": Ixx, "Ixy": Ixy, "Ixz": Ixz,
            "Iyy": Iyy, "Iyz": Iyz, "Izz": Izz,
            "warnings": warnings}


def clamp_physically_feasible(Ixx, Iyy, Izz, Ixy, Ixz, Iyz):
    """
    确保惯性矩阵物理可行：
      1. 对角线 > 0
      2. 全局正定性 (特征值 > 0，等价于三角不等式)
    对于不可行的情况：通过增加对角元素使矩阵正定，警告中显示修正量。
    返回修正后的 (Ixx, Iyy, Izz) 和警告列表。
    """
    import numpy as np

    msgs = []
    Ixx_c, Iyy_c, Izz_c = Ixx, Iyy, Izz

    # Step 1: 对角线 clamp 到正
    for name, val in [("Ixx", Ixx_c), ("Iyy", Iyy_c), ("Izz", Izz_c)]:
        if val <= 0:
            msgs.append(f"{name}={val:.6e} <= 0 → clamp 到 {MIN_INERTIA:.0e}")
    Ixx_c = max(Ixx_c, MIN_INERTIA)
    Iyy_c = max(Iyy_c, MIN_INERTIA)
    Izz_c = max(Izz_c, MIN_INERTIA)

    # Step 2: 构建 3x3 惯性矩阵，检查正定性 (最小特征值 > 0)
    I_mat = np.array([
        [Ixx_c, Ixy, Ixz],
        [Ixy, Iyy_c, Iyz],
        [Ixz, Iyz, Izz_c]
    ])

    eigvals = np.linalg.eigvalsh(I_mat)
    lambda_min = eigvals[0]  # 最小特征值

    if lambda_min <= 0:
        # 将最小特征值抬升到 MIN_EIG，等价于对角加 (MIN_EIG - lambda_min)
        MIN_EIG = 1e-10
        shift = MIN_EIG - lambda_min
        Ixx_c += shift
        Iyy_c += shift
        Izz_c += shift
        msgs.append(
            f"矩阵非正定 (λ_min={lambda_min:.3e}), "
            f"对角 +{shift:.3e} 使 λ_min={MIN_EIG:.0e}"
        )

    return Ixx_c, Iyy_c, Izz_c, msgs


def load_identified_params(yaml_path):
    """从辨识结果 YAML 加载 PCTIKHONOV（或第一个可用算法）参数。"""
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    params_flat = None
    algo_name = None
    for result in data.get("benchmark_results", []):
        algo = result.get("algorithm", "?")
        if algo == "PCTIKHONOV" or params_flat is None:
            raw = result["parameters"]
            params_flat = []
            for item in raw:
                if isinstance(item, list):
                    params_flat.extend(item)
                elif isinstance(item, (int, float)):
                    params_flat.append(item)
            algo_name = algo
            if algo == "PCTIKHONOV":
                break

    if params_flat is None:
        raise ValueError("未找到辨识参数")

    print(f"使用算法: {algo_name}, 共 {len(params_flat)} 个参数")

    # 取前 n_bodies*10 个惯性参数（跳过 armature/damping）
    n_bodies = len(BODY_TO_LINK)
    bodies = []
    for i in range(n_bodies):
        start = i * 10
        bodies.append(params_flat[start:start + 10])
    return bodies


def update_urdf(urdf_path, bodies_com, output_path):
    """
    将 COM 坐标系参数回填到 URDF 的 inertial 块中。
    """
    with open(urdf_path, 'r') as f:
        content = f.read()

    for body_idx, link_name in enumerate(BODY_TO_LINK):
        b = bodies_com[body_idx]

        # 找到 link 开始和对应的 inertial 块
        link_start = content.find(f'<link name="{link_name}">')
        if link_start == -1:
            link_start = content.find(f'<link name="{link_name}"')
        if link_start == -1:
            print(f"  ✗ 找不到 link {link_name}")
            continue

        inertial_start = content.find('<inertial>', link_start)
        inertial_end = content.find('</inertial>', inertial_start)
        if inertial_start == -1 or inertial_end == -1:
            print(f"  ✗ {link_name} 中没有 inertial 块，跳过")
            continue
        inertial_end += len('</inertial>')

        new_inertial = (
            f'    <inertial>\n'
            f'      <origin rpy="0 0 0" xyz="{b["cx"]:.15g} {b["cy"]:.15g} {b["cz"]:.15g}" />\n'
            f'      <mass value="{b["mass"]:.15g}" />\n'
            f'      <inertia ixx="{b["Ixx"]:.15g}" ixy="{b["Ixy"]:.15g}" ixz="{b["Ixz"]:.15g}" '
            f'iyy="{b["Iyy"]:.15g}" iyz="{b["Iyz"]:.15g}" izz="{b["Izz"]:.15g}" />\n'
            f'    </inertial>'
        )
        content = content[:inertial_start] + new_inertial + content[inertial_end:]

        status = "✓" if not b["warnings"] else "⚠"
        print(f"  {status} {link_name} 已更新", end="")
        if b["warnings"]:
            print(f" ({len(b['warnings'])} 个警告)")
            for w in b["warnings"]:
                print(f"      {w}")
        else:
            print()

    with open(output_path, 'w') as f:
        f.write(content)


def main():
    result_yaml = "robots/revoarm_right/result_inertia/revoarm_right_inertia_identification.yaml"
    urdf_path = "URDF/urdf/revoarm_bimanual_revo2.urdf"

    if len(sys.argv) >= 2:
        result_yaml = sys.argv[1]
    if len(sys.argv) >= 3:
        urdf_path = sys.argv[2]

    # 备份原文件
    bak_path = urdf_path + ".bak"
    shutil.copy2(urdf_path, bak_path)
    print(f"原 URDF 已备份到: {bak_path}")

    # 加载辨识参数
    print(f"\n加载辨识结果: {result_yaml}")
    bodies = load_identified_params(result_yaml)

    # body-origin → COM 转换
    print(f"\n{'='*70}")
    print(f"{'Link':<24} {'mass':>8} {'cx':>12} {'cy':>12} {'cz':>12}")
    print(f"{'':24} {'Ixx':>12} {'Iyy':>12} {'Izz':>12}")
    print(f"{'='*70}")

    bodies_com = []
    for i, params in enumerate(bodies):
        b = body_origin_to_com(params)
        bodies_com.append(b)
        link_name = BODY_TO_LINK[i]
        print(f"{link_name:<24} {b['mass']:8.4f} {b['cx']:12.6f} {b['cy']:12.6f} {b['cz']:12.6f}")
        print(f"{'':24} {b['Ixx']:12.6e} {b['Iyy']:12.6e} {b['Izz']:12.6e}")
        for w in b["warnings"]:
            print(f"  ⚠ {w}")
        print()
    print(f"{'='*70}")

    # 统计
    total_warnings = sum(len(b["warnings"]) for b in bodies_com)
    neg_inertias = sum(1 for b in bodies_com if any(w.startswith(("Ixx", "Iyy", "Izz")) for w in b["warnings"]))
    print(f"\n总结: {len(bodies)} 个 link, {total_warnings} 个物理可行性警告")
    if total_warnings > 0:
        print("说明: 辨识结果在力矩预测上是最优的，但可能不完全满足物理可行性约束。")
        print("      负惯性/非正定惯性已被 clamp 到最小正值以保证 URDF 可用。")
        print("      如需更物理可行的结果，可增大 pc_lambda 强制解更靠近 CAD 先验。")

    # 回填 URDF
    print(f"\n回填 URDF: {urdf_path}")
    update_urdf(urdf_path, bodies_com, urdf_path)
    print("\n完成！")


if __name__ == "__main__":
    main()
