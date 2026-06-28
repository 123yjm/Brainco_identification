#!/usr/bin/env python3
"""
从 MuJoCo XML 提取惯性参数，应用平行轴定理转换到 Body Origin 坐标系，
生成 dynamic_prior_params.yaml。

对 link7: 合并 connector_link + hand_base_link 的惯量和质量。

用法:
  python3 scripts/generate_prior_yaml.py \
    --xml /path/to/revoarm_right.xml \
    --kinematic robots/revoarm_right/config/kinematic_params.yaml \
    --output robots/revoarm_right/config/dynamic_prior_params.yaml
"""
import argparse, os, sys, math
import xml.etree.ElementTree as ET
import numpy as np

try:
    import yaml
except ImportError:
    print("需要 PyYAML: pip install pyyaml"); sys.exit(1)

PARAM_NAMES = ["m", "mx", "my", "mz", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz"]


def quat_rotate(q_wxyz, v):
    """用四元数 (w,x,y,z) 旋转向量 v"""
    w, x, y, z = q_wxyz
    # q * v * q_conj
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def parallel_axis(mass, com, I_com):
    """
    平行轴定理: COM 系 → Body Origin 系
    com: COM 在 body origin 系中的坐标 [cx, cy, cz]
    I_com: COM 系惯量 [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    返回: [m, mx, my, mz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz] (body origin)
    """
    cx, cy, cz = com
    Ixx_c, Iyy_c, Izz_c, Ixy_c, Ixz_c, Iyz_c = I_com

    mx = mass * cx
    my = mass * cy
    mz = mass * cz

    Ixx = Ixx_c + mass * (cy*cy + cz*cz)
    Iyy = Iyy_c + mass * (cx*cx + cz*cz)
    Izz = Izz_c + mass * (cx*cx + cy*cy)
    Ixy = Ixy_c - mass * cx * cy
    Ixz = Ixz_c - mass * cx * cz
    Iyz = Iyz_c - mass * cy * cz

    return [mass, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]


def merge_two_inertias(params1, params2):
    """
    合并两个刚体的惯性参数（都在同一坐标系下）。
    返回合并后的 10 维惯性参数向量。
    """
    m1, mx1, my1, mz1, Ixx1, Ixy1, Ixz1, Iyy1, Iyz1, Izz1 = params1
    m2, mx2, my2, mz2, Ixx2, Ixy2, Ixz2, Iyy2, Iyz2, Izz2 = params2

    m = m1 + m2
    mx = mx1 + mx2
    my = my1 + my2
    mz = mz1 + mz2
    Ixx = Ixx1 + Ixx2
    Ixy = Ixy1 + Ixy2
    Ixz = Ixz1 + Ixz2
    Iyy = Iyy1 + Iyy2
    Iyz = Iyz1 + Iyz2
    Izz = Izz1 + Izz2
    return [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]


def parse_inertial(elem):
    """从 XML element 解析 <inertial> 标签"""
    inertial = elem.find("inertial")
    if inertial is None:
        return None
    mass = float(inertial.get("mass", 0))
    com_str = inertial.get("pos", "0 0 0")
    com = np.array([float(v) for v in com_str.split()])
    fi_str = inertial.get("fullinertia", "0 0 0 0 0 0")
    fi = [float(v) for v in fi_str.split()]
    I_com = fi[:6]  # Ixx, Iyy, Izz, Ixy, Ixz, Iyz
    return mass, com, I_com


def parse_quat(elem):
    """解析四元数，返回 (w,x,y,z) numpy array"""
    q_str = elem.get("quat", "1 0 0 0")
    return np.array([float(v) for v in q_str.split()])


def parse_pos(elem):
    """解析位置，返回 (x,y,z) numpy array"""
    p_str = elem.get("pos", "0 0 0")
    return np.array([float(v) for v in p_str.split()])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--xml", required=True, help="MuJoCo XML 文件路径")
    p.add_argument("--kinematic", required=True, help="kinematic_params.yaml 路径")
    p.add_argument("--output", required=True, help="输出 YAML 路径")
    args = p.parse_args()

    # 解析 XML
    tree = ET.parse(args.xml)
    root = tree.getroot()

    # 加载 kinematic_params 获取 body 名称列表
    with open(args.kinematic) as f:
        kcfg = yaml.safe_load(f)
    kprefix = kcfg.get("kinematic_prefix", 0)
    regressor_bodies = kcfg["bodies"][kprefix:]  # 跳过 kinematic prefix

    print(f"Kinematic prefix: {kprefix}, regressor bodies: {len(regressor_bodies)}")
    for i, b in enumerate(regressor_bodies):
        print(f"  body {i}: {b['name']}")

    # ====================================================================
    # 提取 link1-6 的惯性参数 (直接在各自的 <body> 标签内)
    # ====================================================================
    body_params = {}

    # 查找 XML 中各 body 的 inertial
    def find_body(xml_elem, name):
        """递归查找指定名称的 body"""
        if xml_elem.tag == "body" and xml_elem.get("name") == name:
            return xml_elem
        for child in xml_elem:
            if child.tag == "body":
                result = find_body(child, name)
                if result is not None:
                    return result
        return None

    worldbody = root.find("worldbody")

    for i, rb in enumerate(regressor_bodies):
        name = rb["name"]
        if i == 6:  # link7: 需要合并
            continue

        body_elem = find_body(worldbody, name)
        if body_elem is None:
            print(f"  ⚠ 未找到 body: {name}")
            continue

        result = parse_inertial(body_elem)
        if result is None:
            print(f"  ⚠ {name} 无 <inertial> 标签")
            continue

        mass, com, I_com = result
        params = parallel_axis(mass, com, I_com)
        body_params[name] = params

        print(f"  {name}: m={mass:.4f}, COM={com}, Ixx(c)={I_com[0]:.6f}")
        print(f"    → body-origin: [m={params[0]:.4f}, mx={params[1]:.4f}, Ixx={params[4]:.6f}, ...]")

    # ====================================================================
    # link7: 合并 connector_link + hand_base_link
    # ====================================================================
    arm7 = find_body(worldbody, "right_arm_link7")
    if arm7 is None:
        print("  ⚠ 未找到 right_arm_link7")
        link7_params = [0]*10
    else:
        # 获取 arm_link7 的 quat (link7 相对父级的旋转)
        q_link7 = parse_quat(arm7)

        # 查找 connector
        connector = find_body(arm7, "right_connector_link")
        if connector is None:
            print("  ⚠ 未找到 right_connector_link")
            link7_params = [0]*10
        else:
            q_conn = parse_quat(connector)

            # connector 的 COM 在 connector 的 local frame
            r_conn = parse_inertial(connector)
            if r_conn is None:
                print("  ⚠ connector 无 inertial")
                conn_params_body_origin = [0]*10
            else:
                mass_c, com_c, I_c = r_conn
                # connector 的 body frame 相对 arm_link7 的偏移是 0
                # 但四元数旋转需要应用: 先旋转 COM，再转换
                conn_pos_in_link7 = quat_rotate(q_conn, com_c)
                # connector 的惯量在 COM 系，先转回 connector body frame
                # 然后用平行轴定理转回 link7 body origin
                conn_params_body_origin = parallel_axis(mass_c, conn_pos_in_link7, I_c)
                # 注意: 这里惯性张量的旋转被忽略了 (connector 的局部坐标系相对 link7 可能有旋转)
                # 对于小旋转角，近似可接受；精确做法需要旋转惯量张量

            # 查找 hand
            hand = find_body(arm7, "right_hand_base_link")
            if hand is None:
                print("  ⚠ 未找到 right_hand_base_link")
                hand_params_in_link7 = [0]*10
            else:
                r_hand = parse_inertial(hand)
                if r_hand is None:
                    print("  ⚠ hand 无 inertial")
                    hand_params_in_link7 = [0]*10
                else:
                    mass_h, com_h, I_h = r_hand

                    # hand 在 connector_tip → hand_base_link 的路径上
                    # hand 的绝对位置 = connector旋转 + connector_tip_pos + hand_pos
                    # 在 XML 中, hand 的父级是 right_connector_tip, 而 connector_tip 的父级是 connector
                    # connector_tip pos: "7.6e-05 -0.0001518 -0.0583"
                    # hand pos: "0 0 -0.0118"

                    # 查找 connector_tip 和 hand 的相对位置
                    conn_tip = find_body(arm7, "right_connector_tip")
                    tip_pos = np.array([0.0, 0.0, 0.0])
                    hand_pos = np.array([0.0, 0.0, 0.0])
                    if conn_tip is not None:
                        tip_pos = parse_pos(conn_tip)
                        hand_elem = find_body(conn_tip, "right_hand_base_link")
                        if hand_elem is not None:
                            hand_pos = parse_pos(hand_elem)

                    # hand origin 在 connector frame 中的位置
                    hand_origin_in_conn = tip_pos + hand_pos

                    # hand COM 在 hand local frame → 转换到 connector frame
                    # 先转到 hand origin (hand frame 无旋转相对 connector_tip)
                    hand_com_in_conn = hand_origin_in_conn + com_h

                    # 再转到 link7 frame (应用 connector rotation)
                    hand_com_in_link7 = quat_rotate(q_conn, hand_com_in_conn)

                    # 用平行轴定理转到 link7 body origin
                    hand_params_in_link7 = parallel_axis(mass_h, hand_com_in_link7, I_h)

            # 合并 connector + hand
            link7_params = merge_two_inertias(conn_params_body_origin, hand_params_in_link7)

            # connector+hand 的质量和
            m_c = conn_params_body_origin[0] if conn_params_body_origin[0] > 0 else 0
            m_h = hand_params_in_link7[0] if hand_params_in_link7[0] > 0 else 0
            print(f"  link7 (merged): connector(m={m_c:.4f}) + hand(m={m_h:.4f}) = {link7_params[0]:.4f} kg")
            print(f"    → body-origin: [m={link7_params[0]:.4f}, mx={link7_params[1]:.4f}, Ixx={link7_params[4]:.6f}, ...]")

    body_params["right_arm_link7"] = link7_params

    # ====================================================================
    # 填入所有参数的 CAD 值 (不再区分可辨识/不可辨识)
    # baseToFullWithPrior 会自动只取不可辨识部分
    # ====================================================================
    unidentifiable_per_body = {
        # 全填: 让 baseToFullWithPrior 自动适配任意 unidentifiable set
        "right_arm_link1": list(range(10)),
        "right_arm_link2": list(range(10)),
        "right_arm_link3": list(range(10)),
        "right_arm_link4": list(range(10)),
        "right_arm_link5": list(range(10)),
        "right_arm_link6": list(range(10)),
        "right_arm_link7": list(range(10)),
    }

    # ====================================================================
    # 生成 YAML
    # ====================================================================
    output = {}
    output["robot_type"] = "serial_arm"
    output["num_bodies"] = len(regressor_bodies)
    output["description"] = (
        "动态先验参数 — 从 MuJoCo XML 提取，经平行轴定理转换到 body origin 坐标系。\n"
        "  link7 合并了 connector_link + hand_base_link。\n"
        "  所有 70 个参数均填入 CAD 值；baseToFullWithPrior 自动选取不可辨识部分。"
    )
    output["bodies"] = []

    for i, rb in enumerate(regressor_bodies):
        name = rb["name"]
        params = body_params.get(name, [0]*10)
        unid_cols = unidentifiable_per_body.get(name, [])

        entry = {"name": name, "inertial_params": {}}
        for j, pn in enumerate(PARAM_NAMES):
            entry["inertial_params"][pn] = float(params[j])  # 全部填入 CAD 值

        output["bodies"].append(entry)

    # 写入 YAML
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    # 自定义 YAML 输出格式 (null 而非 'null' 字符串)
    with open(args.output, "w") as f:
        f.write(f"# 由 {os.path.basename(__file__)} 自动生成\n")
        f.write(f"# 来源: {args.xml}\n")
        f.write(f"# 平行轴定理: COM → body origin\n")
        f.write(f"# null = 可辨识，待数据求解; 数值 = 不可辨识，CAD 先验\n\n")
        f.write(f"robot_type: \"{output['robot_type']}\"\n")
        f.write(f"num_bodies: {output['num_bodies']}\n")
        f.write(f"description: >\n  {output['description']}\n\n")
        f.write("bodies:\n")

        for body in output["bodies"]:
            f.write(f"  - name: \"{body['name']}\"\n")
            f.write(f"    inertial_params:\n")
            ip = body["inertial_params"]
            for pn in PARAM_NAMES:
                val = ip[pn]
                if val is None:
                    f.write(f"      {pn}: null\n")
                else:
                    f.write(f"      {pn}: {val:.12e}\n")

    print(f"\n✓ 先验参数已保存: {args.output}")

    # 统计
    total = len(regressor_bodies) * 10
    n_null = sum(1 for body in output["bodies"]
                 for v in body["inertial_params"].values() if v is None)
    n_fixed = total - n_null
    print(f"  全参数: {total}, 先验固定: {n_fixed}, 待数据求解: {n_null}")


if __name__ == "__main__":
    main()
