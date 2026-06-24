#!/usr/bin/env python3
"""
txt_traj2csv.py — 将硬件 TXT 轨迹文件转换为激励轨迹 CSV 格式。

csv_traj2txt.py 的逆操作。

输入 TXT 格式:
  22 行 × N 列, 逗号分隔, 无表头
  行  1- 7: q0..q6       (位置, rad)
  行  8-14: qd0..qd6     (速度, rad/s)
  行 15-21: qdd0..qdd6   (加速度, rad/s²)
  行 22  : time          (时间, s)

输出 CSV 格式:
  表头 + N 行 × 29 列
  列: time, q0..q6, qd0..qd6, qdd0..qdd6, tau0..tau6

Usage:
  python3 txt_traj2csv.py input.txt              # 输出到同目录 .csv
  python3 txt_traj2csv.py input.txt output.csv   # 显式指定输出路径
"""

import os
import sys

import numpy as np

DOF = 7
EXPECTED_TXT_ROWS = 22  # 7q + 7qd + 7qdd + 1time
CSV_HEADER = (
    "time,"
    "q0,q1,q2,q3,q4,q5,q6,"
    "qd0,qd1,qd2,qd3,qd4,qd5,qd6,"
    "qdd0,qdd1,qdd2,qdd3,qdd4,qdd5,qdd6,"
    "tau0,tau1,tau2,tau3,tau4,tau5,tau6"
)


def txt_to_csv(txt_path: str, csv_path: str) -> None:
    """将 TXT 轨迹文件转换为 CSV 格式。"""
    # 读取 TXT
    data = np.loadtxt(txt_path, delimiter=",")
    if data.shape[0] != EXPECTED_TXT_ROWS:
        raise ValueError(
            f"TXT 行数应为 {EXPECTED_TXT_ROWS}，实际为 {data.shape[0]}"
        )

    n_cols = data.shape[1]
    print(f"TXT: {data.shape[0]} 行 × {n_cols} 列")

    # 提取各分量
    q   = data[0:7,   :].T    # (N, 7)
    qd  = data[7:14,  :].T    # (N, 7)
    qdd = data[14:21, :].T    # (N, 7)
    t   = data[21,    :]      # (N,)
    tau = np.zeros((n_cols, 7))

    # 堆叠为 29 列
    csv_data = np.column_stack([
        t,
        q[:, 0], q[:, 1], q[:, 2], q[:, 3], q[:, 4], q[:, 5], q[:, 6],
        qd[:, 0], qd[:, 1], qd[:, 2], qd[:, 3], qd[:, 4], qd[:, 5], qd[:, 6],
        qdd[:, 0], qdd[:, 1], qdd[:, 2], qdd[:, 3], qdd[:, 4], qdd[:, 5], qdd[:, 6],
        tau[:, 0], tau[:, 1], tau[:, 2], tau[:, 3], tau[:, 4], tau[:, 5], tau[:, 6],
    ])

    # 写入 CSV
    np.savetxt(
        csv_path, csv_data,
        delimiter=",", header=CSV_HEADER, comments="",
        fmt="%.15e",
    )

    dt = np.mean(np.diff(t))
    print(f"CSV 已保存: {csv_path}")
    print(f"  行数: {csv_data.shape[0]}, 列数: {csv_data.shape[1]}")
    print(f"  时间范围: [{t[0]:.2f}, {t[-1]:.2f}] s, dt={dt:.4f}s")
    print(f"  q  范围: [{q.min():.4f}, {q.max():.4f}] rad")
    print(f"  qd 范围: [{qd.min():.4f}, {qd.max():.4f}] rad/s")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {os.path.basename(__file__)} <input.txt> [output.csv]")
        sys.exit(1)

    txt_path = sys.argv[1]

    if len(sys.argv) >= 3:
        csv_path = sys.argv[2]
    else:
        base, _ = os.path.splitext(txt_path)
        csv_path = base + ".csv"

    print(f"输入 TXT: {txt_path}")
    print(f"输出 CSV: {csv_path}")
    txt_to_csv(txt_path, csv_path)


if __name__ == "__main__":
    main()
