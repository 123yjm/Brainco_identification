#!/usr/bin/env python3
"""
读取 recorded_trajectory_*.txt 文件并绘制对比图
用法：
    python3 plot_recorded_trajectory.py recorded_trajectory_20260127_214641.txt
    或直接修改下面的 filename 变量
"""

import sys
import numpy as np
import matplotlib.pyplot as plt


def _make_subplot_grid(n_joints, cols=3):
    rows = int(np.ceil(n_joints / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows), sharex=True)
    axes = np.atleast_1d(axes).flatten()
    return fig, axes


def _hide_unused_axes(axes, used_count):
    for ax in axes[used_count:]:
        ax.set_visible(False)


def load_data(filename):
    """读取 txt 文件，返回各个数组"""
    try:
        data = np.loadtxt(filename, delimiter=",")
    except Exception as e:
        print(f"读取文件失败: {e}")
        sys.exit(1)
    
    total_cols = data.shape[1]
    if (total_cols - 1) % 6 != 0:
        print(f"列数不匹配！总列数应满足 1 + 6*N，实际 {total_cols} 列")
        sys.exit(1)

    n_joints = (total_cols - 1) // 6
    if n_joints <= 0:
        print(f"关节数无效：{n_joints}")
        sys.exit(1)

    idx = 1
    t = data[:, 0]                            # 时间戳 (col 0)
    q = data[:, idx:idx + n_joints]           # actual position
    idx += n_joints
    dq = data[:, idx:idx + n_joints]          # actual velocity
    idx += n_joints
    tau = data[:, idx:idx + n_joints]         # actual effort
    idx += n_joints
    qr = data[:, idx:idx + n_joints]          # reference position
    idx += n_joints
    dqr = data[:, idx:idx + n_joints]         # reference velocity
    idx += n_joints
    ddqr = data[:, idx:idx + n_joints]        # reference acc
    
    return t, q, dq, tau, qr, dqr, ddqr, n_joints


def plot_trajectory(filename):
    t, q, dq, tau, qr, dqr, ddqr, n_joints = load_data(filename)
    
    joint_names = [f'joint{i+1}' for i in range(n_joints)]
    
    # 图1：位置对比 qr vs q
    fig1, axes1 = _make_subplot_grid(n_joints)
    fig1.suptitle('Position Tracking: Reference (qr) vs Actual (q)', fontsize=16)
    
    for i in range(n_joints):
        ax = axes1[i]
        ax.plot(t, qr[:, i], 'b-', label='qr (reference)', linewidth=1.5)
        ax.plot(t, q[:, i],  'r--', label='q (actual)', linewidth=1.2)
        ax.set_title(joint_names[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.grid(True)
        ax.legend(loc='upper right', fontsize='small')

    _hide_unused_axes(axes1, n_joints)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    
    # 图2：速度对比 dqr vs dq
    fig2, axes2 = _make_subplot_grid(n_joints)
    fig2.suptitle('Velocity Tracking: Reference (dqr) vs Actual (dq)', fontsize=16)
    
    for i in range(n_joints):
        ax = axes2[i]
        ax.plot(t, dqr[:, i], 'b-', label='dqr (reference)', linewidth=1.5)
        ax.plot(t, dq[:, i],  'r--', label='dq (actual)', linewidth=1.2)
        ax.set_title(joint_names[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.grid(True)
        ax.legend(loc='upper right', fontsize='small')

    _hide_unused_axes(axes2, n_joints)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    
    # 图3：电流（effort / tau）对比
    # 注意：这里没有参考电流，所以只画实际 tau
    fig3, axes3 = _make_subplot_grid(n_joints)
    fig3.suptitle('Actual Joint Effort (Current / Torque)', fontsize=16)
    
    for i in range(n_joints):
        ax = axes3[i]
        ax.plot(t, tau[:, i], 'g-', label='tau (actual effort)', linewidth=1.5)
        ax.set_title(joint_names[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Effort (Nm or A)')
        ax.grid(True)
        ax.legend(loc='upper right', fontsize='small')

    _hide_unused_axes(axes3, n_joints)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        # 如果不传参数，可以在这里手动指定文件名
        # filename = "/home/xyd/BrainCo/revoarm_developing/revoarm_teleoperation/hardware_control_debug/logs/recorded_trajectory_20260314_163353.txt"  # ← 修改成你的实际文件名
        filename = "/home/ubuntu/Desktop/brainco_identification/robots/single_right/data_inertia/single_right_excitation_trajectory_20260623_141852.txt"
    print(f"正在读取文件: {filename}")
    plot_trajectory(filename)