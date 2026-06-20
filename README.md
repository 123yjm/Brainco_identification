# Brainco Identification

串联机械臂（7-DOF）动力学参数辨识工具链，包含四个独立可执行模块：

| 可执行文件 | 入口 | 功能 |
|---|---|---|
| `filter_data` | `src/app/main_filter.cpp` | 巴特沃斯低通滤波数据预处理 |
| `filter_and_solve` | `src/app/main_filter_and_solve.cpp` | 滤波 + 辨识一站式管线（无中间 CSV） |
| `get_traj` | `src/app/main_traj.cpp` | 傅里叶级数激励轨迹优化 |
| `identify` | `src/app/main_solve.cpp` | 动力学参数辨识（6 种算法） |

---

## 依赖

- **C++17**、**Eigen3**、**yaml-cpp**、**NLopt**（`libnlopt-cxx-dev`）
- GUI（可选）：**Qt5 Widgets**
- Python 绑图（可选）：`matplotlib`、`pandas`

```bash
sudo apt install libeigen3-dev libyaml-cpp-dev libnlopt-cxx-dev
```

## 编译

```bash
cd build && cmake .. && make -j$(nproc)
```

生成的可执行文件在 `build/` 下：`filter_data`、`get_traj`、`identify`。

---

## 1. 巴特沃斯滤波器 — `filter_data`

将 43 列原始采样 `.txt` 低通滤波后输出 29 列 `.csv`，供辨识模块使用。

```
用法: ./filter_data [选项]

选项:
  --config <yaml>    配置文件 (默认: config/butterworth_filter.yaml)
  --input <txt>      覆盖输入 .txt 路径
  --output <csv>     覆盖输出 .csv 路径
  --passband <Hz>    覆盖通带频率
  --stopband <Hz>    覆盖阻带频率
  --help             打印帮助信息
```

### 配置文件 `config/butterworth_filter.yaml`

```yaml
io:
  input_txt: data/revoarm_right_data_read_excitation_56.12_0618.txt
  output_csv: result/filtered_data.csv

filter:
  sampling_frequency_hz: 100
  passband_hz: 2           # 越低曲线越平滑
  stopband_hz: 4
  passband_ripple_db: 1
  stopband_attenuation_db: 50
```

- 滤波算法：5 阶数字 Butterworth 低通 + 零相位 `filtfilt`
- 加速度通过中心差分计算
- 输出 CSV 与 MATLAB 脚本 `data/Step1_butterworth.m` 结果一致

### 可视化

```bash
python3 scripts/plot_filtered_data.py --input result/filtered_data.csv
```

生成 4 张子图（q / qd / qdd / tau），保存在 `graph/` 目录。

---

## 2. 滤波 + 辨识一站式管线 — `filter_and_solve`

省去中间 CSV 文件，原始 `.txt` 读入后直接内存滤波 → 内存辨识 → 输出结果 `.yaml`。

```
用法: ./filter_and_solve [选项]

选项:
  --filter-config <yaml>  滤波配置 (默认: config/butterworth_filter.yaml)
  --solve-config <yaml>   辨识配置 (默认: config/identification.yaml)
  --passband <Hz>         覆盖通带频率
  --stopband <Hz>         覆盖阻带频率
  --algo <name>           覆盖辨识算法
  --output <yaml>         覆盖结果输出路径
  --no-damping            禁用阻尼项辨识
  --help                  打印帮助信息
```

数据流: `.txt(43列)` → `filtfilt + 中心差分` → `ExperimentData` → `辨识算法` → `结果.yaml`

---

## 3. 激励轨迹优化器 — `get_traj`

使用 5 阶傅里叶级数生成 D-optimal 激励轨迹，最大化观测矩阵的行列式以提高辨识精度。

```
用法: ./get_traj [--config <yaml>]
```

### 配置文件 `config/excite_trajectory.yaml`

- 优化目标：最大化 `det(WᵀW)`（D-optimal 准则）
- 优化器：NLopt（COBYLA 全局 + SLSQP 局部精修，多起点策略）
- 约束：初始位置/速度/加速度边界、关节限位

### 输出

- `result/excitation_trajectory.csv` — 包含 time, q, qd, qdd, tau 的完整轨迹

---

## 4. 动力学参数辨识器 — `identify`

从滤波后的轨迹数据辨识串联机械臂的惯性参数和关节阻尼系数。

```
用法: ./identify [选项]

选项:
  --config <yaml>          配置文件 (默认: config/identification.yaml)
  --data <csv>             覆盖数据文件路径
  --algo <name>            指定算法 (OLS/WLS/IRLS/TLS/EKF/NLS_FRICTION)
  --output <yaml>          覆盖结果输出路径
  --kinematic-params <yaml> 覆盖运动学参数文件
  --no-damping             禁用阻尼项辨识
```

### 支持的算法

| 名称 | 说明 |
|---|---|
| `OLS` | 普通最小二乘 |
| `WLS` | 加权最小二乘 |
| `IRLS` | 迭代重加权最小二乘 |
| `TLS` | 总体最小二乘 |
| `EKF` | 扩展卡尔曼滤波 |
| `NLS_FRICTION` | 非线性摩擦项 LM 优化 |

### 配置文件 `config/identification.yaml`

- 机器人运动学参数：`config/kinematic_params.yaml`
- 结果输出：`result/identification.yaml`（含各算法 RMSE、最大误差、辨识参数）

---

## 数据流

```
原始采样 .txt (43列)
    │
    ▼
filter_data ──→ 滤波 .csv (29列)
    │
    ├──→ identify ──→ 辨识结果 .yaml
    │
    └──→ get_traj ──→ 激励轨迹 .csv
```

---

## 目录结构

```
brainco_identification/
├── main.cpp              # 辨识入口
├── main_traj.cpp         # 轨迹优化入口
├── main_filter.cpp       # 滤波器入口
├── CMakeLists.txt
├── include/              # 头文件
│   ├── butterworth_filter.hpp
│   ├── csv_io.hpp
│   ├── algorithms.hpp
│   ├── dynamics_regressor.hpp
│   ├── serial_arm_regressor.hpp
│   ├── trajectory_optimizer.hpp
│   └── ...
├── src/                  # 实现文件
│   ├── butterworth_filter.cpp
│   ├── csv_io.cpp
│   ├── algorithms.cpp
│   ├── serial_arm_regressor.cpp
│   ├── trajectory_optimizer.cpp
│   └── qt/               # GUI
├── config/               # YAML 配置文件
│   ├── butterworth_filter.yaml
│   ├── identification.yaml
│   ├── excite_trajectory.yaml
│   └── kinematic_params.yaml
├── scripts/              # 辅助脚本
│   └── plot_filtered_data.py
├── data/                 # 数据 & MATLAB 预处理脚本
├── graph/                # 绑图输出
├── result/               # 辨识 & 轨迹输出
└── doc/                  # 文档
```
