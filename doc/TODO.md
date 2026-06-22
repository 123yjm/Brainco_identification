# brainco_identification 开发计划

## ✅ 已完成

### 架构 — 机器人目录注册式
- [x] 统一 `--robot <dir>` CLI，四个入口函数一致
- [x] `robot_utils` 公共工具（路径推导、文件名生成）
- [x] 配置文件精简，删除冗余 `robot`/`data_file`/`output_file`/`io` 字段
- [x] 输出文件自动带机器人前缀（如 `revoarm_right_identification.yaml`）
- [x] `robot_type`（工厂用）与 `robot_name`（文件命名用）分离

### 滤波器 — `filter_data`
- [x] 5 阶 Butterworth 低通 + 零相位 `filtfilt` + 中心差分加速度
- [x] 43 列 `.txt` → 29 列 `.csv`，输出与 MATLAB 一致
- [x] `config/butterworth_filter.yaml` 可调参
- [x] Python 绑图脚本 `scripts/plot_filtered_data.py`

### 辨识器 — `identify`
- [x] 6 种算法：OLS / WLS / IRLS / TLS / EKF / NLS_FRICTION
- [x] `RegressorFactory` 工厂模式，YAML 驱动 `kinematic_params.yaml`
- [x] 阻尼项可选（`--no-damping`）

### 激励轨迹优化 — `get_traj`
- [x] 5 阶傅里叶级数 D-optimal 优化
- [x] NLopt COBYLA + SLSQP 多起点策略

### 一站式管线 — `filter_and_solve`
- [x] 原始 `.txt` → 内存滤波 → 内存辨识 → 结果 `.yaml`，无中间 CSV

### QT 上位机 — `identify_gui`
- [x] 图形化界面，通过 QProcess 调用 `identify` 二进制

### 编译
- [x] `compile.sh` 一键编译

---

## 📋 待完成

### 1. 真机代码标准化
**目标**：修改真机端代码，使轨迹读取和采样数据输出的格式统一为标准 CSV。

- [ ] **1.1 轨迹读取**：真机激励轨迹输入改为标准 CSV 格式（当前为 43 列无表头 `.txt`，需对齐 `get_traj` 输出的 CSV 格式）
- [ ] **1.2 采样输出**：真机数据采样输出改为带表头 CSV（当前为 43 列无表头 `.txt`，需改为与 `filter_data` 输入兼容的格式，含列名说明）
- [ ] **1.3 格式文档**：在 `doc/` 下补充 CSV 格式规范文档，列明每列含义、单位、取值范围

### 2. revoarm_left 配置
- [ ] 编写 `robots/revoarm_left/config/kinematic_params.yaml`
- [ ] 采集 `revoarm_left` 的激励轨迹数据
- [ ] 全管线测试（滤波 → 辨识 → 轨迹优化）

### 3. 代码清理
- [ ] `CLOE` 存根类：当前回退到 OLS，如无实现计划则删除
- [ ] 旧 regressor 中未使用的 body Jacobian 方法清理

---

## 依赖

| 依赖 | 状态 | 说明 |
|------|------|------|
| Eigen 3 | ✅ | 矩阵运算 |
| yaml-cpp | ✅ | 所有配置文件解析 |
| NLopt | ✅ | 轨迹优化 |
| Qt5 Widgets | ✅ | GUI 上位机 |
| matplotlib + pandas | ✅ | Python 绑图脚本 |

---

## 里程碑

```
✅ 滤波器 + 辨识器 + 轨迹优化器 + GUI      已完成
✅ 机器人目录注册式架构大改                  已完成
📋 真机 CSV 格式标准化                      待完成
📋 revoarm_left 配置与测试                   待完成
📋 代码清理                                 待完成
```
