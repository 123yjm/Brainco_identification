# Brainco Identification

串联机械臂（7-DOF）动力学参数辨识工具链，包含八个独立可执行模块：

| 可执行文件 | 入口 | 功能 |
|---|---|---|
| `filter_data` | `src/app/main_filter.cpp` | 巴特沃斯低通滤波数据预处理 |
| `filter_and_solve_inertia` | `src/app/main_filter_and_solve_inertia.cpp` | 滤波 + 惯性辨识一站式管线（无中间 CSV） |
| `get_excite_traj` | `src/app/main_get_excite_traj.cpp` | 傅里叶级数激励轨迹优化 |
| `get_friction_traj` | `src/app/main_get_friction_traj.cpp` | 摩擦力辨识轨迹生成（分时匀速） |
| `calc_rank` | `src/app/main_calc_rank.cpp` | 观测矩阵秩分析 |
| `identify_inertia` | `src/app/main_solve_inertia.cpp` | 惯性参数辨识（5 种算法） |
| `identify_inertia_PC` | `src/app/main_solve_inertia_PC.cpp` | **PC 惯量辨识**（先验中心 Tikhonov + 摩擦修正） |
| `identify_friction` | `src/app/main_solve_friction.cpp` | 摩擦力辨识 — Coulomb + Viscous 全数据点 OLS |

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
./filter_and_solve_inertia     --robot robots/revoarm_right
./get_excite_traj      --robot robots/revoarm_right
./get_friction_traj    --robot robots/revoarm_right
./identify_inertia     --robot robots/revoarm_right
./identify_inertia_PC  -r revoarm_right
./identify_friction    --robot robots/revoarm_right
```

### 机器人目录结构

```
robots/revoarm_right/
├── config/
│   ├── butterworth_filter.yaml        # 滤波器参数
│   ├── inertia_identification.yaml    # 惯性辨识配置
│   ├── inertia_PC_identification.yaml # PC 惯量辨识配置 (先验 + λ)
│   ├── friction_identification.yaml   # 摩擦力辨识参数
│   ├── excite_trajectory.yaml         # 激励轨迹优化参数
│   ├── friction_trajectory.yaml       # 摩擦轨迹参数
│   └── kinematic_params.yaml          # 运动学/动力学参数
├── data_inertia/                      # 惯性激励轨迹采集数据 (*.txt)
├── data_friction/                     # 摩擦轨迹采集数据 (*.csv)
├── result_inertia/                    # 惯性辨识输出
│   ├── revoarm_right_filtered_data.csv
│   ├── revoarm_right_inertia_identification.yaml
│   └── revoarm_right_excitation_trajectory.csv
├── result_inertia_PC/                 # PC 惯量辨识输出
│   └── revoarm_right_inertia_identification.yaml
└── result_friction/                   # 摩擦辨识输出
    ├── revoarm_right_friction_trajectory.csv
    └── revoarm_right_friction_identification.yaml
```

## 编译

```bash
cd build && cmake .. && make -j$(nproc)
```

---

## 辨识流程

完整的动力学参数辨识分三步进行，每一步建立在前一步的基础上。这种分步策略不是随意为之，而是由动力学方程的结构决定的。

### 为什么需要三步辨识

串联机械臂的关节力矩方程可写为：

$$\tau = Y_{\text{full}}(q, \dot{q}, \ddot{q}) \cdot \beta_{\text{full}}$$

其中 $\beta_{\text{full}}$ 包含惯性参数（70 个）、粘性阻尼（7 个）和 Coulomb+Viscous 摩擦系数（14 个），共 91 个参数。一次性全参数辨识面临三个互相耦合的困难：

1. **摩擦与惯性的耦合**：激励轨迹同时激发惯性力和摩擦力，W 矩阵对应列之间存在相关性，导致参数估计方差放大。
2. **物理可行性无法保证**：无约束 OLS 求解的惯性参数经并行轴定理转换到 COM 坐标系后，惯量张量可能不正定（负特征值），导致无法加载到仿真器中。
3. **重力项需要精确 β**：摩擦辨识需要从低速匀速运动中扣除重力力矩，而重力项 $Y(q, 0, 0) \cdot \beta$ 的精度完全依赖于惯性参数 β 的准确性。

三步分步辨识正是针对这三个困难设计的：

- **第一步（粗略辨识）**：用大范围激励轨迹辨识惯性+damping，得到对重力项足够精确的 β，**不要求物理一致性**。
- **第二步（摩擦辨识）**：利用第一步的重力前馈精确扣除重力，从低速匀速运动中提取纯摩擦力，辨识 Coulomb+Viscous 系数。
- **第三步（PC 辨识）**：将第二步的摩擦力从激励轨迹力矩中扣除，再用 CAD 先验正则化求解惯性参数，**强制物理一致性**。

每一步消去一个不确定因素，最终获得完整的、物理可行的动力学参数集。

### 操作流程

```
1. 创建机器人 → 填入运动学参数
        │
        ▼
2. 生成轨迹 → 采集数据
   ├── get_excite_traj    → data_inertia/*.txt
   └── get_friction_traj  → data_friction/*.csv
        │
        ▼
3. 粗略惯性辨识 (data_inertia)
   filter_data + identify_inertia
   → 获得不PC的 β (惯性+damping)，用于重力前馈
        │
        ▼
4. 摩擦力辨识 (data_friction + 重力前馈)
   identify_friction
   → 获得 Fc, Fv (14 参数)
        │
        ▼
5. PC 惯性辨识 (data_inertia - friction + CAD 先验)
   identify_inertia_PC
   → 获得物理可行的 β_PC (70 惯性参数)
        │
        ▼
   完成：β_PC (惯性) + damping (7) + friction (14) = 全部 91 参数
```

### 各步骤详解

**步骤 1 — 创建机器人**：在 `robots/<name>/config/kinematic_params.yaml` 中填入 DH 参数（bodies 列表、joint_axis 等），并从 CAD 软件导出各 link 的 COM 惯量（mass、COM 位置、惯性张量），用于后续先验。

**步骤 2 — 生成轨迹**：`get_excite_traj` 生成傅里叶级数激励轨迹（充分激发各参数），`get_friction_traj` 生成分时匀速摩擦轨迹（各关节依次匀速正反转）。将轨迹部署到实机采集力矩数据。

**步骤 3 — 粗略辨识**：从激励轨迹数据中直接辨识惯性+damping（无摩擦修正，无先验约束）。这组参数物理上不一致（部分 COM 惯量为负），但用于重力前馈已经足够——因为在重力项计算中，误差主要来源于一阶矩（mx, my, mz），而质量跟踪精度通常较高。

**步骤 4 — 摩擦辨识**：利用步骤 3 的 β 从摩擦数据中逐点扣除重力（$\tau_{\text{ng}} = \tau - Y(q,0,0)\beta$），在匀速段对纯摩擦力做 OLS 回归，得到每个关节的 Coulomb 系数 Fc 和 Viscous 系数 Fv。

**步骤 5 — PC 辨识**：将摩擦修正（$\tau_c = \tau - \tau_{\text{friction}}$）应用于激励轨迹力矩，结合 CAD 先验进行逐参数加权的 Tikhonov 正则化。惯量张量分量赋予较大正则化权重以强制物理可行性，质量分量给予较小权重让其充分跟踪数据。

### 为什么步骤 5 需要单独的可执行文件

PC 辨识的配置、输入和输出与普通辨识完全不同：
- **配置文件**：`inertia_PC_identification.yaml`（含 CAD 先验 + 三层正则化权重），而非 `inertia_identification.yaml`
- **额外输入**：摩擦辨识结果（Fc, Fv）
- **输出目录**：`result_inertia_PC/`，而非 `result_inertia/`
- **参数范围**：仅 70 惯性参数（不含 damping，因阻尼已在步骤 3 中辨识）

因此 `identify_inertia_PC` 作为独立可执行文件，使用 `-r <name>` 短参数，自动加载所需配置。

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

## 2. 滤波 + 惯性辨识一站式 — `filter_and_solve_inertia`

```bash
./filter_and_solve_inertia --robot robots/revoarm_right [--algo <name>]  [--no-damping]
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
./identify_inertia --robot robots/revoarm_right [--algo <name>]  [--no-damping]
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
regularization: 1      # 1=开启 Tikhonov 正则化 (λ=1e-6), 0=关闭
damping: true          # 是否辨识粘性阻尼
```

CLI `--algo` / `--no-damping` 可覆盖 YAML 配置。

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
        damping (N_DOF)
        ]
```

默认参数数：77 = 7×10 惯性 + 7 damping

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

## 7. PC 惯量辨识器 — `identify_inertia_PC`

```bash
./identify_inertia_PC -r <robot> [--pc-lam-m <val>] [--pc-lam-c <val>] [--pc-lam-i <val>]
```

PC（Physical Consistent）惯量辨识：从激励轨迹数据中扣除摩擦力，使用先验中心 Tikhonov 正则化求解仅 70 个惯性参数（不含 armature/damping）。

### 前置条件

1. 已运行 `identify_friction`，获得 `result_friction/<robot>_friction_identification.yaml`
2. 已创建 `config/inertia_PC_identification.yaml`（含 CAD 先验 body-origin 参数 + 正则化权重）

### 辨识流程

```
W * beta = tau - tau_friction
min ||W*beta - (tau - tau_f)||² + Σ wᵢ(betaᵢ - beta_priorᵢ)²
```

1. 加载 `result_inertia/<robot>_filtered_data.csv`（激励轨迹数据）
2. 加载 `config/inertia_PC_identification.yaml`（先验 + 正则化参数）
3. 加载 `result_friction/<robot>_friction_identification.yaml`（Fc, Fv）
4. 逐时刻计算摩擦力 $\tau_{f,j} = F_{c,j} \cdot \text{sign}(\dot{q}_j) + F_{v,j} \cdot \dot{q}_j$ 并扣除
5. 构建观测矩阵 W（仅 70 惯性参数，`ParamFlags::NONE`）
6. 用逐参数权重的先验中心 Tikhonov 求解

### 配置文件 `config/inertia_PC_identification.yaml`

```yaml
# 正则化权重 (三层分离)
pc_lambda_mass: 100               # 质量 m（可给较小值）
pc_lambda_com: 1000000            # 质心 mx, my, mz
pc_lambda_inertia: 50000000       # 惯量 Ixx..Izz（需较大值）

# CAD 先验 (body-origin 坐标系, 7 body × 10 参数)
# 由 URDF COM 惯量经 InertialParams::fromBody() 并行轴定理转换
prior_inertia:
  - [mass, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]  # body 0
  - ...
```

### 输出

输出到 `result_inertia_PC/<robot>_inertia_identification.yaml`，格式与普通辨识一致。

### 三层正则化权重

每个 body 的 10 个参数分为三层，各自独立控制正则化强度：

| 层 | 参数位置 | CLI 选项 | 说明 |
|----|---------|---------|------|
| λ_mass | [0] m | `--pc-lam-m` | 质量，易辨识，可给小值 |
| λ_com | [1-3] mx, my, mz | `--pc-lam-c` | COM 位置，需中等约束防止平行轴定理破坏 |
| λ_inertia | [4-9] Ixx..Izz | `--pc-lam-i` | 惯量张量，难辨识，需大值拉向先验 |

### 辅助脚本

```bash
# 从 URDF COM 惯量生成 body-origin 先验
python3 scripts/backfill_urdf.py   # 辨识结果回填 URDF（含物理可行性自修正）
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

`filter_and_solve_inertia` 是 `filter_data` + `identify_inertia` 的一站式版本，滤波在内存中完成不落盘：

```
data_inertia/*.txt ──► [filter_and_solve_inertia] ──► result_inertia/*_inertia_identification.yaml
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

### PC 惯量辨识管线

```
result_inertia/*_filtered_data.csv ──► [identify_inertia_PC] ──► result_inertia_PC/*_inertia_identification.yaml
        ▲                                    ▲
config/inertia_PC_identification.yaml        result_friction/*_friction_identification.yaml
(先验 + λ)                                  (Fc, Fv)
```

### 各可执行文件输入来源

| 可执行文件 | 输入文件 | 路径 | 必须 |
|---|---|---|---|
| `filter_data` | 原始数据 | `<robot_dir>/data_inertia/*.txt`（自动取首个） | ✅ |
| | 滤波器配置 | `<robot_dir>/config/butterworth_filter.yaml` | ✅ |
| `filter_and_solve_inertia` | 原始数据 | `<robot_dir>/data_inertia/*.txt` | ✅ |
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
| `identify_inertia_PC` | 已滤波数据 | `<robot_dir>/result_inertia/<robot>_filtered_data.csv` | ✅ |
| | 运动学参数 | `<robot_dir>/config/kinematic_params.yaml` | ✅ |
| | PC 辨识配置 | `<robot_dir>/config/inertia_PC_identification.yaml` | ✅ |
| | 摩擦辨识结果 | `<robot_dir>/result_friction/<robot>_friction_identification.yaml` | ✅ |
| `identify_friction` | 已采集数据 | `<robot_dir>/data_friction/*.csv`（自动取首个） | ✅ |
| | 惯性参数 β | `<robot_dir>/result_inertia/<robot>_inertia_identification.yaml` | ✅ |
| | 摩擦轨迹配置 | `<robot_dir>/config/friction_trajectory.yaml` | ✅ |
| | 摩擦辨识配置 | `<robot_dir>/config/friction_identification.yaml` | ✅ |

路径通过 `--robot <dir>` 或 `-r <name>` 自动推导。`identify_inertia_PC` 额外使用 `-r` 短参数直接指定机器人名称。

### 辨识参数构成

| 组别 | 数量 | 说明 |
|------|------|------|
| 惯性参数 | N_BODIES × 10 | 每个关节连杆: m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz |

| damping | N_DOF | 粘性阻尼（`inertia_identification.yaml` 中 `damping: false` 可关闭） |

对于 7-DOF revoarm: 默认 77 参数，关闭 damping 后降至 70。
