#!/usr/bin/env python3
"""
plot_filtered_data.py — 绘制滤波后的激励轨迹数据

读取 29 列滤波 CSV（time, q0..q6, qd0..qd6, qdd0..qdd6, tau0..tau6），
生成 4 张 Figure 保存到 graph/ 目录:
  figure1_q.png   — 关节位置
  figure2_qd.png  — 关节速度
  figure3_qdd.png — 关节加速度
  figure4_tau.png — 关节力矩

用法:
  python3 scripts/plot_filtered_data.py --input result/filtered_data.csv
  python3 scripts/plot_filtered_data.py                        # 使用脚本内默认路径
"""

import argparse
import math
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd

# ---------------------------------------------------------------------------
# 默认输入路径（可直接修改）
# ---------------------------------------------------------------------------
# DEFAULT_INPUT = "/home/ubuntu/Desktop/brainco_identification/result/filtered_data.csv"
DEFAULT_INPUT = "/home/ubuntu/Desktop/brainco_identification/data/revoarm_filtered_data_condnum_56.12_0618.csv"
# ---------------------------------------------------------------------------
# 输出目录（相对于项目根目录，而非当前工作目录）
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "graph")

# ---------------------------------------------------------------------------
# 四组信号的列名前缀、y 轴标签、输出文件名
# ---------------------------------------------------------------------------
PLOT_CONFIG = [
    {
        "prefix": "q",
        "ylabel": "jointpos / rad",
        "title": "Joint Position (q)",
        "filename": "figure1_q.png",
    },
    {
        "prefix": "qd",
        "ylabel": "jointvel / rad/s",
        "title": "Joint Velocity (qd)",
        "filename": "figure2_qd.png",
    },
    {
        "prefix": "qdd",
        "ylabel": "jointacc / rad/s²",
        "title": "Joint Acceleration (qdd)",
        "filename": "figure3_qdd.png",
    },
    {
        "prefix": "tau",
        "ylabel": "torque / (N·m or A)",
        "title": "Joint Torque (tau)",
        "filename": "figure4_tau.png",
    },
]


def main():
    parser = argparse.ArgumentParser(
        description="绘制滤波后的激励轨迹数据"
    )
    parser.add_argument(
        "--input",
        type=str,
        default=None,
        help=f"滤波 CSV 文件路径 (默认: {DEFAULT_INPUT})",
    )
    args = parser.parse_args()

    csv_path = args.input or DEFAULT_INPUT

    if not os.path.exists(csv_path):
        print(f"错误: 文件不存在 — {csv_path}", file=sys.stderr)
        sys.exit(1)

    # ---- 读取 CSV -----------------------------------------------------------
    print(f"读取: {csv_path}")
    df = pd.read_csv(csv_path)

    # 自动检测 DOF 数量: 总列数 = 1(time) + 4 * n_dof
    n_cols = len(df.columns)
    n_dof = (n_cols - 1) // 4
    print(f"列数: {n_cols}, DOF: {n_dof}")

    time = df["time"].to_numpy()

    # ---- 创建输出目录 --------------------------------------------------------
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ---- 绘制四张图 ----------------------------------------------------------
    # 子图网格: 行数 ceil(n_dof/3), 列数 3（匹配 MATLAB 3×3 布局）
    n_cols_sub = min(n_dof, 3)
    n_rows_sub = math.ceil(n_dof / n_cols_sub)

    for cfg in PLOT_CONFIG:
        prefix = cfg["prefix"]
        fig, axes = plt.subplots(n_rows_sub, n_cols_sub, figsize=(14, 10))
        fig.suptitle(cfg["title"], fontsize=13)

        # 将 axes 展平为一维，方便遍历
        if n_rows_sub == 1 and n_cols_sub == 1:
            axes_flat = [axes]
        else:
            axes_flat = axes.flatten()

        for j in range(n_dof):
            ax = axes_flat[j]
            col_name = f"{prefix}{j}"
            if col_name in df.columns:
                ax.plot(time, df[col_name].to_numpy(), linewidth=0.6)
            else:
                print(f"  警告: 列 '{col_name}' 不存在，跳过")

            ax.set_title(f"{prefix}{j}", fontsize=10)
            ax.set_xlabel("time / s")
            ax.set_ylabel(cfg["ylabel"])
            ax.grid(True, alpha=0.3)

        # 隐藏多余的子图
        for j in range(n_dof, len(axes_flat)):
            axes_flat[j].set_visible(False)

        fig.tight_layout()
        out_path = os.path.join(OUTPUT_DIR, cfg["filename"])
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"  已保存: {out_path}")

    print(f"\n完成 — {len(PLOT_CONFIG)} 张图已保存到 {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
