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

## 机器人注册机制

所有入口函数统一使用 `--robot <dir>` 指向机器人目录，配置、数据、输出路径均自动推导。

```bash
./filter_data        --robot robots/revoarm_right
./filter_and_solve   --robot robots/revoarm_right
./get_traj           --robot robots/revoarm_right
./identify           --robot robots/revoarm_right
```

### 机器人目录结构

```
robots/revoarm_right/
├── config/
│   ├── butterworth_filter.yaml   # 滤波器参数
│   ├── identification.yaml       # 辨识参数
│   ├── excite_trajectory.yaml    # 轨迹优化参数
│   └── kinematic_params.yaml     # 运动学/动力学参数
├── data/                         # 原始 .txt 数据
└── result/                       # 输出文件（自动带机器人前缀）
    ├── revoarm_right_filtered_data.csv
    ├── revoarm_right_identification.yaml
    └── revoarm_right_excitation_trajectory.csv
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

自动找 `data/*.txt` 作为输入，输出 `result/<robot>_filtered_data.csv`。

### 可视化

```bash
python3 scripts/plot_filtered_data.py --input robots/revoarm_right/result/revoarm_right_filtered_data.csv
```

---

## 2. 滤波 + 辨识一站式 — `filter_and_solve`

```bash
./filter_and_solve --robot robots/revoarm_right [--algo <name>] [--no-damping]
```

数据流: `.txt` → 内存滤波 → `ExperimentData` → 辨识 → `<robot>_identification.yaml`

---

## 3. 激励轨迹优化器 — `get_traj`

```bash
./get_traj --robot robots/revoarm_right
```

输出: `result/<robot>_excitation_trajectory.csv`

---

## 4. 动力学参数辨识器 — `identify`

```bash
./identify --robot robots/revoarm_right [--algo <name>] [--no-damping]
```

读已有 CSV: `result/<robot>_filtered_data.csv`，输出: `result/<robot>_identification.yaml`

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
