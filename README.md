# Brainco Identification

串联机械臂（7-DOF）动力学参数辨识工具链，包含四个独立可执行模块：

| 可执行文件 | 入口 | 功能 |
|---|---|---|
| `filter_data` | `src/app/main_filter.cpp` | 巴特沃斯低通滤波数据预处理 |
| `filter_and_solve` | `src/app/main_filter_and_solve.cpp` | 滤波 + 辨识一站式管线（无中间 CSV） |\n| `get_excite_traj` | `src/app/main_get_excite_traj.cpp` | 傅里叶级数激励轨迹优化 |
| `get_friction_traj` | `src/app/main_get_friction_traj.cpp` | 摩擦力辨识轨迹生成 |
| `calc_rank` | `src/app/main_calc_rank.cpp` | 观测矩阵秩分析 |
| `identify_inertia` | `src/app/main_solve_inertia.cpp` | 惯性参数辨识（6 种算法） |
| `identify_friction` | `src/app/main_solve_friction.cpp` | 摩擦力辨识（预留） |

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
./filter_data        --robot robots/revoarm_right
./filter_and_solve   --robot robots/revoarm_right
./get_excite_traj    --robot robots/revoarm_right
./identify_inertia   --robot robots/revoarm_right
```

### 机器人目录结构

```
robots/revoarm_right/
├── config/
│   ├── butterworth_filter.yaml      # 滤波器参数
│   ├── inertia_identification.yaml   # 惯性辨识参数
│   ├── friction_identification.yaml  # 摩擦力辨识参数
│   ├── excite_trajectory.yaml       # 激励轨迹优化参数
│   ├── friction_trajectory.yaml     # 摩擦轨迹参数
│   └── kinematic_params.yaml        # 运动学/动力学参数
├── data_inertia/                    # 惯性激励轨迹采集数据 (*.txt)
├── data_friction/                   # 摩擦轨迹采集数据 (*.txt)
├── result_inertia/                  # 惯性辨识输出（自动带机器人前缀）
│   ├── revoarm_right_filtered_data.csv
│   ├── revoarm_right_inertia_identification.yaml
│   └── revoarm_right_excitation_trajectory.csv
└── result_friction/                 # 摩擦辨识输出
    └── revoarm_right_friction_trajectory.csv
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

## 2. 滤波 + 辨识一站式 — `filter_and_solve`

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

## 数据流与输入文件

### 完整管线

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

### 各可执行文件输入来源

| 可执行文件 | 输入文件 | 路径 | 必须 |
|---|---|---|---|
| `filter_data` | 原始数据 | `<robot_dir>/data_inertia/*.txt`（自动取首个） | ✅ |
| | 滤波器配置 | `<robot_dir>/config/butterworth_filter.yaml` | ✅ |
| `filter_and_solve` | 原始数据 | `<robot_dir>/data_inertia/*.txt` | ✅ |
| | 滤波器配置 | `<robot_dir>/config/butterworth_filter.yaml` | ✅ |
| | 运动学参数 | `<robot_dir>/config/kinematic_params.yaml` | ✅ |
| | 辨识配置 | `<robot_dir>/config/inertia_identification.yaml` | 可选 |
| `identify_inertia` | 已滤波数据 | `<robot_dir>/result_inertia/<robot>_filtered_data.csv` | ✅ |
| | 运动学参数 | `<robot_dir>/config/kinematic_params.yaml` | ✅ |
| | 辨识配置 | `<robot_dir>/config/inertia_identification.yaml` | 可选 |

路径通过 `--robot <dir>` 自动推导（`config/`、`data_inertia/`、`data_friction/`、`result_inertia/`、`result_friction/` 子目录）。

### 辨识参数构成

| 组别 | 数量 | 说明 |
|------|------|------|
| 惯性参数 | N_BODIES × 10 | 每个关节连杆: m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz |
| armature | N_DOF | 电机转子反映惯量（`inertia_identification.yaml` 中 `armature: false` 可关闭） |
| damping | N_DOF | 粘性阻尼（`inertia_identification.yaml` 中 `damping: false` 可关闭） |

对于 7-DOF revoarm: 默认 84 参数，关闭 armature+damping 后降至 70。
