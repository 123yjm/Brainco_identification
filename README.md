# Brainco Identification

串联机械臂（7-DOF）动力学参数辨识工具链，包含七个独立可执行模块：

| 可执行文件 | 入口 | 功能 |
|---|---|---|
| `filter_data` | `src/app/main_filter.cpp` | 巴特沃斯低通滤波数据预处理 |
| `filter_and_solve` | `src/app/main_filter_and_solve.cpp` | 滤波 + 惯性辨识一站式管线（无中间 CSV） |
| `get_excite_traj` | `src/app/main_get_excite_traj.cpp` | 傅里叶级数激励轨迹优化 |
| `get_friction_traj` | `src/app/main_get_friction_traj.cpp` | 摩擦力辨识轨迹生成（分时匀速） |
| `calc_rank` | `src/app/main_calc_rank.cpp` | 观测矩阵秩分析 |
| `identify_inertia` | `src/app/main_solve_inertia.cpp` | 惯性参数辨识（6 种算法） |
| `identify_friction` | `src/app/main_solve_friction.cpp` | 摩擦力辨识 — Coulomb + Viscous 全数据点 OLS |
| `identify_PF` | `src/app/main_solve_PF.cpp` | 物理可行性辨识 — 固定先验 + 降维 OLS |

---

## 依赖

- **C++17**、**Eigen3**、**yaml-cpp**、**NLopt**（`libnlopt-cxx-dev`）
- GUI（可选）：**Qt5 Widgets**
- Python 绑图（可选）：`matplotlib`、`pandas`

```bash
sudo apt install libeigen3-dev libyaml-cpp-dev libnlopt-cxx-dev
```

## 机器人注册机制

所有入口函数统一使用 `--robot <dir>` 指向机器人目录，配置、数据、输出路径均自动推导。

```bash
./filter_data          --robot robots/revoarm_right
./filter_and_solve     --robot robots/revoarm_right
./get_excite_traj      --robot robots/revoarm_right
./get_friction_traj    --robot robots/revoarm_right
./identify_inertia     --robot robots/revoarm_right
./identify_friction    --robot robots/revoarm_right
./identify_PF          --robot robots/revoarm_right
```

### 机器人目录结构

```
robots/revoarm_right/
├── config/
│   ├── butterworth_filter.yaml        # 滤波器参数
│   ├── inertia_identification.yaml    # 惯性辨识参数
│   ├── friction_identification.yaml   # 摩擦力辨识参数
│   ├── excite_trajectory.yaml         # 激励轨迹优化参数
│   ├── friction_trajectory.yaml       # 摩擦轨迹参数
│   └── kinematic_params.yaml          # 运动学/动力学参数
├── data_inertia/                      # 惯性激励轨迹采集数据 (*.txt)
├── data_friction/                     # 摩擦轨迹采集数据 (*.csv)
├── result_inertia/                    # 惯性辨识输出（自动带机器人前缀）
│   ├── revoarm_right_filtered_data.csv
│   ├── revoarm_right_inertia_identification.yaml
│   └── revoarm_right_excitation_trajectory.csv
├── result_friction/                   # 摩擦辨识输出
│   ├── revoarm_right_friction_trajectory.csv
│   └── revoarm_right_friction_identification.yaml
└── result_pf/                          # PF 辨识输出
    └── revoarm_right_pf_identification.yaml
```

## 编译

```bash
cd build && cmake .. && make -j$(nproc)
```

---

## 1. 巴特沃斯滤波器 — `filter_data`

```bash
./filter_data --robot robots/revoarm_right [--passband <Hz>] [--stopband <Hz>]
```

自动找 `data_inertia/*.txt` 作为输入，输出 `result_inertia/<robot>_filtered_data.csv`。

### 可视化

```bash
python3 scripts/plot_filtered_data.py --input robots/revoarm_right/result_inertia/revoarm_right_filtered_data.csv
```

---

## 2. 滤波 + 惯性辨识一站式 — `filter_and_solve`

```bash
./filter_and_solve --robot robots/revoarm_right [--algo <name>] [--no-armature] [--no-damping]
```

数据流: `.txt` (data_inertia) → 内存滤波 → `ExperimentData` → 辨识 → `result_inertia/<robot>_inertia_identification.yaml`

---

## 3. 激励轨迹优化器 — `get_excite_traj`

```bash
./get_excite_traj --robot robots/revoarm_right
```

输出: `result_inertia/<robot>_excitation_trajectory.csv`

---

## 4. 惯性参数辨识器 — `identify_inertia`

```bash
./identify_inertia --robot robots/revoarm_right [--algo <name>] [--no-armature] [--no-damping]
```

读已有 CSV: `result_inertia/<robot>_filtered_data.csv`，输出: `result_inertia/<robot>_inertia_identification.yaml`

### 支持的算法

| 编号 | 名称 | 说明 |
|------|------|------|
| 1 | `OLS` | 普通最小二乘 |
| 2 | `WLS` | 加权最小二乘 |
| 3 | `IRLS` | 迭代重加权最小二乘 |
| 4 | `TLS` | 总体最小二乘 |
| 5 | `EKF` | 扩展卡尔曼滤波 |
| 8 | `NLS_FRICTION` | 非线性摩擦项 LM 优化 |
| 0 | benchmark | 运行全部算法 |

### 配置文件 `config/inertia_identification.yaml`

```yaml
# 算法编号 (algorithm):
#   1 = OLS  2 = WLS  3 = IRLS  4 = TLS  5 = EKF  8 = NLS_FRICTION  0 = 全部
algorithm: 1
regularization: 1      # 1=开启 Tikhonov 正则化, 0=关闭
armature: true         # 是否辨识转子惯量
damping: true          # 是否辨识粘性阻尼
```

CLI `--algo` / `--no-armature` / `--no-damping` 可覆盖 YAML 配置。

### 输出格式 `result_inertia/<robot>_inertia_identification.yaml`

参数以流式 YAML 数组输出，按 10 个/行分组：

```yaml
benchmark_results:
  - algorithm: "OLS"
    torque_rmse: 0.205
    torque_max_error: 1.025
    parameters:
        [
        惯性参数 (N_BODIES×10, 每行10个),
        (空行)
        armature (N_DOF),
        (空行)
        damping (N_DOF)
        ]
```

默认参数数：84 = 7×10 惯性 + 7 armature + 7 damping

---

## 5. 摩擦轨迹生成器 — `get_friction_traj`

```bash
./get_friction_traj --robot robots/revoarm_right
```

生成各关节分时匀速运动轨迹，用于摩擦力辨识数据采集。每个关节依次执行 4 段匀速运动：+v1, -v1, +v2, -v2，非运动关节锁定在 `q_init`。加减速阶段使用梯形（或三角形）速度 profile。

输出: `result_friction/<robot>_friction_trajectory.csv`（CSV 格式与激励轨迹完全一致：`time, q0..q6, qd0..qd6, qdd0..qd6, tau0..tau6`）

### 配置文件 `config/friction_trajectory.yaml`

```yaml
sampling_frequency: 100   # 输出采样频率 (Hz)
acceleration: 2.0         # 加减速阶段加速度 (rad/s²)

q_init:      [0, 0, 0, 0, 0, 0, 0]     # 各关节起始位置 (rad)
q_target_v1: [-0.7, 0.7, -0.5, ...]     # 低速 v1 的目标位置 (rad)
q_target_v2: [-1.5, 1.5, -1.0, ...]     # 高速 v2 的目标位置 (rad)

v1: 0.1   # 低速 (rad/s)
v2: 1.0   # 高速 (rad/s)
```

---

## 6. 摩擦力辨识器 — `identify_friction`

```bash
./identify_friction --robot robots/revoarm_right
```

从 `data_friction/*.csv` 加载匀速段数据，使用已辨识的惯性参数 β 逐点精确扣除重力后，对每个关节独立做全数据点 OLS 线性回归，求解 Coulomb + Viscous 摩擦系数。

### 摩擦模型

$$\tau_{\text{friction}} = F_c \cdot \text{sign}(\dot{q}) + F_v \cdot \dot{q}$$

### 辨识流程

1. 加载 `data_friction/*.csv`（自动取首个 CSV）
2. 加载已辨识的惯性参数 β（来自 `result_inertia/<robot>_inertia_identification.yaml`）
3. 根据 `friction_trajectory.yaml` 计算各关节匀速段时间窗口
4. 对每个关节，收集所有窗口内的数据点：
   - 逐点计算重力力矩 $\tau_{\text{grav}}(q) = Y(q, 0, 0) \cdot \beta$
   - 扣除重力: $\tau_{\text{ng}} = \tau - \tau_{\text{grav}}$
5. 对每个关节独立 OLS 回归: $[\text{sign}(\dot{q}), \dot{q}] \cdot [F_c, F_v]^T = \tau_{\text{ng}}$
6. 计算 RMSE: $\text{RMSE} = \sqrt{\frac{1}{n}\sum(\hat{\tau}_{\text{friction}} - \tau_{\text{ng}})^2}$

### 配置文件 `config/friction_identification.yaml`

```yaml
interval_shrink: 0.8   # 匀速段时间窗口缩放因子（0~1），补偿控制延迟
```

### 输出格式 `result_friction/<robot>_friction_identification.yaml`

```yaml
calibration_date: "2026-06-24 21:27:15"
robot: "revoarm_right"
method: "ols_gravity_exact_removal"
interval_shrink: 0.8
parameters:
  [
    2.345, 1.023,
    1.876, 0.954,
    ...
  ]
```

每关节一行两个参数: `Fc (Nm), Fv (Nm·s/rad)`。若辨识结果为负值，保存时自动 clamp 到 0。

### 终端输出示例

```
═══════════════════════════════════════════
  摩擦力辨识 — Coulomb + Viscous
  (重力精确扣除 + 全数据点回归)
═══════════════════════════════════════════
机器人:           revoarm_right
数据:             robots/revoarm_right/data_friction/friction_trajectory_xxx.csv
样本数:           6943
v1 / v2:          0.1 / 1 rad/s
interval_shrink:  0.8

j0: 1234 个数据点
j1: 1187 个数据点
...

-------------------------------------------------------
关节      Fc (Nm)        Fv (Nm·s/rad)   残差 RMSE   数据点数
-------------------------------------------------------
j0           2.345678        1.023456        0.123456
j1           1.876543        0.954321        0.098765
...
-------------------------------------------------------
```

---

## 7. 物理可行性辨识器 — `identify_PF`

```bash
./identify_PF --robot robots/revoarm_right
```

使用已辨识的摩擦参数（Fc, Fv）和 `kinematic_params.yaml` 中的动力学先验值，构建降维线性回归系统，对未知惯性/armature 参数做 OLS 求解。已知先验参数固定不动。

### 辨识流程

```
1. 加载 result_inertia/<robot>_filtered_data.csv（与 identify_inertia 共用）
2. 加载摩擦参数: Fc, Fv（来自 result_friction/<robot>_friction_identification.yaml）
3. 逐点扣除摩擦: tau_net = tau - friction(qd, Fc, Fv)
4. 构建完整回归矩阵 W_full（惯性 + armature，不含 damping）
5. 获取先验向量 beta_prior 和已知掩码 mask
   - 显式设定于 kinematic_params.yaml 的字段 → mask=true（已知）
   - 未设定的字段 → mask=false（未知，待求解）
6. 分离已知/未知列: W_known, W_unknown
7. 右移已知项: tau_reduced = tau_net - W_known * beta_known
8. OLS 求解: beta_unknown = (W_unknown^T W_unknown + λI)^-1 W_unknown^T tau_reduced
9. 合并: beta_full = beta_known ∪ beta_unknown
```

### 先验值来源

`kinematic_params.yaml` 中每个 body 的可选先验字段（分轴独立设定）：

| 字段 | 类型 | 说明 |
|---|---|---|
| `mass` | double | 质量 (kg) |
| `com_x`, `com_y`, `com_z` | double | 质心分量 (m)，可单独约束 |
| `Ixx`, `Iyy`, `Izz` | double | 惯量对角分量 |
| `Ixy`, `Ixz`, `Iyz` | double | 惯量交叉分量 |

> **判定规则**：字段**显式出现**在 YAML 中即视为已知（即使值为 0），未出现则视为未知需辨识。通过 `computeParameterMask()` 自动推导平行轴定理贡献：若 `com_y` 和 `com_z` 均已知，则 `Ixx` 平行轴部分 `m*(cy²+cz²)` 可计算 → `Ixx` 标记为已知。

### 输出格式

保存至 `result_pf/<robot>_pf_identification.yaml`：

```yaml
calibration_date: "2026-06-25 21:54:14"
robot: "revoarm_right"
method: "physically_feasible_ols"
evaluation_method: "torque_residual_rmse"
known_parameter_count: 49
unknown_parameter_count: 28
torque_rmse: 0.226
torque_max_error: 1.610
parameters:
    [
    惯性参数 (N_BODIES×10, 每行10个),

    armature (N_DOF)
    ]
```

---

## 数据流与输入文件

### 惯性辨识管线

```
data_inertia/*.txt (原始数据, 43列无header)     Config YAML
         │                                           │
         ├──► [filter_data] ◄── butterworth_filter.yaml
         │         │                                 │
         │         ▼                                 │
         │   result_inertia/*_filtered_data.csv      │
         │         │                                 │
         │         ▼                                 │
         └──► [identify_inertia] ◄── kinematic_params.yaml
                   │            inertia_identification.yaml (可选)
                   ▼
            result_inertia/*_inertia_identification.yaml
```

`filter_and_solve` 是 `filter_data` + `identify_inertia` 的一站式版本，滤波在内存中完成不落盘：

```
data_inertia/*.txt ──► [filter_and_solve] ──► result_inertia/*_inertia_identification.yaml
                    ▲
              butterworth_filter.yaml
              kinematic_params.yaml
              inertia_identification.yaml (可选)
```

### 摩擦辨识管线

```
config/friction_trajectory.yaml ──► [get_friction_traj] ──► result_friction/*_friction_trajectory.csv
                                                                        │
                                                          (实机回放采集数据)
                                                                        │
                                                                        ▼
                                                              data_friction/*.csv
                                                                        │
result_inertia/*_inertia_identification.yaml ──┐                       │
                                                ├──► [identify_friction]
config/friction_trajectory.yaml ───────────────┘        │
config/friction_identification.yaml ────────────────────┘
                                                                        │
                                                                        ▼
                                                              result_friction/*_friction_identification.yaml
```

### PF 辨识管线

```
result_inertia/*_filtered_data.csv ──────────┐
                                              │
result_friction/*_friction_identification.yaml ──┤
                                                  ├──► [identify_PF] ──► result_pf/*_pf_identification.yaml
config/kinematic_params.yaml (先验值) ──────────┤
config/inertia_identification.yaml ─────────────┘
```

### 各可执行文件输入来源

| 可执行文件 | 输入文件 | 路径 | 必须 |
|---|---|---|---|
| `filter_data` | 原始数据 | `<robot_dir>/data_inertia/*.txt`（自动取首个） | ✅ |
| | 滤波器配置 | `<robot_dir>/config/butterworth_filter.yaml` | ✅ |
| `filter_and_solve` | 原始数据 | `<robot_dir>/data_inertia/*.txt` | ✅ |
| | 滤波器配置 | `<robot_dir>/config/butterworth_filter.yaml` | ✅ |
| | 运动学参数 | `<robot_dir>/config/kinematic_params.yaml` | ✅ |
| | 辨识配置 | `<robot_dir>/config/inertia_identification.yaml` | 可选 |
| `get_excite_traj` | 轨迹配置 | `<robot_dir>/config/excite_trajectory.yaml` | ✅ |
| | 运动学参数 | `<robot_dir>/config/kinematic_params.yaml` | ✅ |
| `get_friction_traj` | 轨迹配置 | `<robot_dir>/config/friction_trajectory.yaml` | ✅ |
| | 运动学参数 | `<robot_dir>/config/kinematic_params.yaml` | ✅ |
| `identify_inertia` | 已滤波数据 | `<robot_dir>/result_inertia/<robot>_filtered_data.csv` | ✅ |
| | 运动学参数 | `<robot_dir>/config/kinematic_params.yaml` | ✅ |
| | 辨识配置 | `<robot_dir>/config/inertia_identification.yaml` | 可选 |
| `identify_friction` | 已采集数据 | `<robot_dir>/data_friction/*.csv`（自动取首个） | ✅ |
| | 惯性参数 β | `<robot_dir>/result_inertia/<robot>_inertia_identification.yaml` | ✅ |
| | 摩擦轨迹配置 | `<robot_dir>/config/friction_trajectory.yaml` | ✅ |
| | 摩擦辨识配置 | `<robot_dir>/config/friction_identification.yaml` | ✅ |
| `identify_PF` | 已滤波数据 | `<robot_dir>/result_inertia/<robot>_filtered_data.csv` | ✅ |
| | 摩擦参数 | `<robot_dir>/result_friction/<robot>_friction_identification.yaml` | ✅ |
| | 运动学参数 | `<robot_dir>/config/kinematic_params.yaml` | ✅ |
| | 辨识配置 | `<robot_dir>/config/inertia_identification.yaml` | 可选 |

路径通过 `--robot <dir>` 自动推导（`config/`、`data_inertia/`、`data_friction/`、`result_inertia/`、`result_friction/`、`result_pf/` 子目录）。

### 辨识参数构成

| 组别 | 数量 | 说明 |
|------|------|------|
| 惯性参数 | N_BODIES × 10 | 每个关节连杆: m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz |
| armature | N_DOF | 电机转子反映惯量（`inertia_identification.yaml` 中 `armature: false` 可关闭） |
| damping | N_DOF | 粘性阻尼（`inertia_identification.yaml` 中 `damping: false` 可关闭） |

对于 7-DOF revoarm: 默认 84 参数，关闭 armature+damping 后降至 70。
