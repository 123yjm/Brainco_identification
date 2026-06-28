#!/usr/bin/env python3
"""
csv_traj2txt.py — 将激励轨迹 CSV 文件转换为硬件 TXT 轨迹格式。

txt_traj2csv.py 的逆操作。

输入 CSV 格式:
  表头 + N 行 × 29 列
  列: time, q0..q6, qd0..qd6, qdd0..qdd6, tau0..tau6

输出 TXT 格式:
  22 行 × N 列, 逗号分隔, 无表头
  行  1- 7: q0..q6       (位置, rad)
  行  8-14: qd0..qd6     (速度, rad/s)
  行 15-21: qdd0..qdd6   (加速度, rad/s²)
  行 22  : time          (时间, s)

Usage:
  python3 csv_traj2txt.py input.csv              # 输出到同目录 .txt
  python3 csv_traj2txt.py input.csv output.txt   # 显式指定输出路径
"""

import os
import sys

import numpy as np

DOF = 7
EXPECTED_CSV_COLS = 29  # time + 7q + 7qd + 7qdd + 7tau
EXPECTED_TXT_ROWS = 22  # 7q + 7qd + 7qdd + 1time


def csv_to_txt(csv_path: str, txt_path: str) -> None:
    """将 CSV 轨迹文件转换为 TXT 格式。"""
    # 读取 CSV（跳过表头）
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    if data.shape[1] != EXPECTED_CSV_COLS:
        raise ValueError(
            f"CSV 列数应为 {EXPECTED_CSV_COLS}，实际为 {data.shape[1]}"
        )

    n_rows = data.shape[0]
    print(f"CSV: {n_rows} 行 × {data.shape[1]} 列")

    # 提取各分量并转置为 (DOF, N)
    t   = data[:, 0]           # (N,)
    q   = data[:, 1:8].T       # (7, N)
    qd  = data[:, 8:15].T      # (7, N)
    qdd = data[:, 15:22].T     # (7, N)

    # 按 TXT 布局堆叠: 22 行 × N 列
    txt_data = np.vstack([
        q,              # 行  1- 7: q0..q6
        qd,             # 行  8-14: qd0..qd6
        qdd,            # 行 15-21: qdd0..qdd6
        t.reshape(1, -1),  # 行 22: time
    ])

    # 写入 TXT
    np.savetxt(
        txt_path, txt_data,
        delimiter=",", fmt="%.15e",
    )

    dt = np.mean(np.diff(t))
    print(f"TXT 已保存: {txt_path}")
    print(f"  形状: {txt_data.shape[0]} 行 × {txt_data.shape[1]} 列")
    print(f"  时间范围: [{t[0]:.2f}, {t[-1]:.2f}] s, dt={dt:.4f}s")
    print(f"  q  范围: [{q.min():.4f}, {q.max():.4f}] rad")
    print(f"  qd 范围: [{qd.min():.4f}, {qd.max():.4f}] rad/s")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {os.path.basename(__file__)} <input.csv> [output.txt]")
        sys.exit(1)

    csv_path = sys.argv[1]

    if len(sys.argv) >= 3:
        txt_path = sys.argv[2]
    else:
        base, _ = os.path.splitext(csv_path)
        txt_path = base + ".txt"

    print(f"输入 CSV: {csv_path}")
    print(f"输出 TXT: {txt_path}")
    csv_to_txt(csv_path, txt_path)


if __name__ == "__main__":
    main()
