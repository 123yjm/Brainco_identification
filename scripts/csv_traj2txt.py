#!/usr/bin/env python3
"""
csv_traj2txt.py — 将 get_excite_traj 生成的激励轨迹 CSV 转换为硬件控制所需 TXT 格式。

输入 CSV 格式 (get_excite_traj 输出):
  表头 + 101 行 × 29 列
  列: time, q0..q6, qd0..qd6, qdd0..qdd6, tau0..tau6
  采样: dt=0.1s, 单周期 T=10s

输出 TXT 格式 (硬件控制读取):
  22 行 × 3003 列, 无表头, 逗号分隔
  行  1- 7: 关节 0-6 位置 (rad)
  行  8-14: 关节 0-6 速度 (rad/s)
  行 15-21: 关节 0-6 加速度 (rad/s^2)
  行 22  : 时间 (s)
  采样: dt=0.01s, 三周期 T_total=30s
"""

import numpy as np
import sys
import os

# ---------------------------------------------------------------------------
# 参数
# ---------------------------------------------------------------------------
CSV_PATH = "/home/ubuntu/Desktop/brainco_identification/robots/single_right/result/single_right_excitation_trajectory.csv"
TXT_PATH = "/home/ubuntu/Desktop/brainco_identification/robots/single_right/result/single_right_excitation_trajectory.txt"

DOF = 7
SAMPLING_TIME = 10.0    # 激励轨迹运行时长 (s)
TRAJECTORY_FREQ = 100   # 轨迹发布频率 (Hz)
DT_OUT = 1.0 / TRAJECTORY_FREQ  # 输出采样间隔 (s)
NUM_PERIODS = 3         # 重复周期数
EXPECTED_CSV_COLS = 29  # time + 7q + 7qd + 7qdd + 7tau

# ---------------------------------------------------------------------------
# 读取 CSV
# ---------------------------------------------------------------------------
def load_csv(path):
    with open(path, 'r') as f:
        header = f.readline().strip()
    print(f"CSV 表头: {header}")

    data = np.loadtxt(path, delimiter=',', skiprows=1)
    if data.shape[1] != EXPECTED_CSV_COLS:
        raise ValueError(f"CSV 列数应为 {EXPECTED_CSV_COLS}，实际为 {data.shape[1]}")

    t_in = data[:, 0]                    # 时间 (101 点, 0:0.1:10)
    q_in   = data[:, 1:8]                # 位置 (101 × 7)
    qd_in  = data[:, 8:15]               # 速度
    qdd_in = data[:, 15:22]              # 加速度
    # tau 列忽略 (全零)

    print(f"CSV 采样点数: {len(t_in)}, 时间范围: [{t_in[0]:.2f}, {t_in[-1]:.2f}] s")
    return t_in, q_in, qd_in, qdd_in

# ---------------------------------------------------------------------------
# 三周期重复 (CSV 已由 get_excite_traj 以 trajectory_frequency 输出，无需插值)
# ---------------------------------------------------------------------------
def repeat_periods(y_one, n_periods):
    """repmat 风格重复 n_periods 个周期（不移除周期边界重叠点）。"""
    return np.vstack([y_one] * n_periods)

# ---------------------------------------------------------------------------
# 生成时间向量
# ---------------------------------------------------------------------------
def generate_time_vector(n_periods):
    n_per_period = int(SAMPLING_TIME / DT_OUT) + 1     # 1001
    n_total = n_periods * n_per_period                  # 3003 (repmat 风格)
    t = np.linspace(0, n_periods * SAMPLING_TIME, n_total)
    return t

# ---------------------------------------------------------------------------
# 写入 TXT
# ---------------------------------------------------------------------------
def write_txt(path, q, qd, qdd, t):
    """
    写入 22 行 × N 列 TXT 文件。
    行  1- 7: q[0]..q[6]
    行  8-14: qd[0]..qd[6]
    行 15-21: qdd[0]..qdd[6]
    行 22  : t
    """
    rows = []

    # 位置行
    for j in range(DOF):
        rows.append(q[:, j])
    # 速度行
    for j in range(DOF):
        rows.append(qd[:, j])
    # 加速度行
    for j in range(DOF):
        rows.append(qdd[:, j])
    # 时间行
    rows.append(t)

    with open(path, 'w') as f:
        for row in rows:
            line = ','.join(f'{v:.15g}' for v in row)
            f.write(line + '\n')

    print(f"TXT 已保存: {path}")
    print(f"  行数: {len(rows)}, 列数: {len(rows[0])}")
    print(f"  位置范围: [{q.min():.4f}, {q.max():.4f}] rad")
    print(f"  速度范围: [{qd.min():.4f}, {qd.max():.4f}] rad/s")

# ===========================================================================
if __name__ == '__main__':
    csv_path = sys.argv[1] if len(sys.argv) > 1 else CSV_PATH
    txt_path = sys.argv[2] if len(sys.argv) > 2 else TXT_PATH

    print(f"输入 CSV: {csv_path}")
    print(f"输出 TXT: {txt_path}")

    # 1. 读取
    t_in, q_in, qd_in, qdd_in = load_csv(csv_path)

    # 2. 三周期重复 (CSV 已由 get_excite_traj 以 trajectory_frequency 输出，直接 repmat)
    print(f"\n重复 {NUM_PERIODS} 周期 ({len(t_in)} 点/周期)...")
    q_out   = repeat_periods(q_in,   NUM_PERIODS)
    qd_out  = repeat_periods(qd_in,  NUM_PERIODS)
    qdd_out = repeat_periods(qdd_in, NUM_PERIODS)

    # 3. 时间向量
    t_out = generate_time_vector(NUM_PERIODS)

    print(f"输出采样点: {len(t_out)}")

    # 4. 写入
    write_txt(txt_path, q_out, qd_out, qdd_out, t_out)
