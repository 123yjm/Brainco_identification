# brainco_identification 开发计划

## Phase 0 — 代码清理

**目标**：删除无用代码，为后续重构减负。

### 0.1 待清理项
- [ ] `CLOE` 类 ([algorithms.hpp:74-79](include/algorithms.hpp#L74-L79) / [algorithms.cpp]): 当前回退到 OLS 的存根，如果短期内无实现计划则删除
- [ ] `computeBodyOriginJacobian` / `computeBodyJacobian` / `computeBodyOriginJacobianDerivative` / `computeBodyJacobianDerivative` ([revoarm_new_regressor.hpp:51-58](include/revoarm_new_regressor.hpp#L51-L58)): 这四个 public/private 方法在 `computeBodyRegressorBlock` 中未使用（regressor block 内联了相同的逻辑），Phase 1 重构后将彻底删除
- [ ] `parseSimpleYaml` ([main.cpp:67-103](main.cpp#L67-L103)): Phase 2 引入正式 yaml-cpp 后替换掉这个手写解析器
- [ ] `poseToTransform` / `skew` ([revoarm_new_regressor.cpp:146-160](src/revoarm_new_regressor.cpp#L146-L160)): Phase 1 引入 Pinocchio 后，`poseToTransform` 不再需要；`skew` 在 regressor 公式中仍需保留
- [ ] `computeBodyTransforms` / `computeBodyCOM` ([revoarm_new_regressor.cpp:166-202](src/revoarm_new_regressor.cpp#L166-L202)): Phase 1 后删除，由 Pinocchio 替代
- [ ] 确认 `config/identification.yaml` 中的 `robot` 字段当前未被使用（实际硬编码了 `RevoarmNewRegressor`），Phase 2 将激活该字段

### 0.2 当前架构问题记录（重构前需了解）
- `main.cpp` 直接实例化 `RevoarmNewRegressor`，无工厂抽象 → Phase 2 引入 `RegressorFactory`
- FK 在每 body 的 `computeBodyRegressorBlock` 中被重复计算（每 body 调用 3 次 FK）→ Phase 1 一次性解决
- `regularization` 配置值 `1` 被 `std::stoi` 解析为 bool，语义不明确 → Phase 2 统一配置格式

---

## Phase 1 — Pinocchio 替换手搓 Jacobian

**目标**：用 Pinocchio 的 FK / J / dJ/dt / frame acceleration 替换手动几何计算 + 数值微分，保留回归矩阵公式不变。

**核心原则**：
- 参数化方式不变（Body 原点系标准惯性参数 `[m, mx, my, mz, Ixx..Izz]`）
- `computeBodyRegressorBlock` 的回归公式（line 420-453）不变
- armature / damping 列不变
- 仅替换运动学计算后端

### 1.1 添加 Pinocchio 依赖
- [ ] `CMakeLists.txt`: 添加 `find_package(pinocchio REQUIRED)` 和 `target_link_libraries(... pinocchio::pinocchio)`
- [ ] 确认编译通过（系统需安装 pinocchio ≥ 2.6）

### 1.2 创建 `PinocchioRegressor` 类
- [ ] **新建** `include/pinocchio_regressor.hpp`
- [ ] **新建** `src/pinocchio_regressor.cpp`

#### 类设计

```cpp
class PinocchioRegressor {
public:
  static constexpr std::size_t N_DOF = 7;
  static constexpr std::size_t N_BODIES = 7;

  PinocchioRegressor();

  // 接口与 RevoarmNewRegressor 完全一致
  VectorXd computeParameterVector(ParamFlags flags) const;
  std::size_t numParameters(ParamFlags flags) const;
  MatrixXd computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                   const VectorXd &qdd, ParamFlags flags) const;
  MatrixXd computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                     const MatrixXd &Qdd, ParamFlags flags) const;
  std::vector<std::string> getParameterNames(ParamFlags flags) const;

private:
  // --- 沿用现有的参数管理 ---
  std::array<RigidBody, N_BODIES + 2> bodies_;
  Vector3d gravity_{0, 0, -9.81};
  void initBodies();  // 暂时复用现有硬编码（Phase 2 改为 YAML 加载）

  // --- 新增 Pinocchio 成员 ---
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<pinocchio::FrameIndex> body_frame_ids_;

  void buildPinocchioModel();  // 从 bodies_ 构建 pinocchio::Model（纯 C++ API）

  // 核心：替换运动学后端，回归公式不变
  MatrixXd computeBodyRegressorBlock(std::size_t body_idx,
                                      const VectorXd &q, const VectorXd &qd,
                                      const VectorXd &qdd) const;
  static Matrix3d skew(const Vector3d &v);  // 保留
};
```

#### `buildPinocchioModel()` 实现要点

```
bodies_[0] (base_link)       → JointModelFixed (世界原点)
bodies_[1] (right_base_link) → JointModelFixed + SE3(quat, pos)
bodies_[2..8] (link1..link7) → JointModelRevolute(joint_axis) + SE3(quat, pos)
                                + appendBodyToJoint(Inertia(mass, com, I_com))
                                + addJointFrame → 存 FrameIndex
```

**不需要 URDF，所有参数直接从 `bodies_` 取。**

#### `computeBodyRegressorBlock()` 实现要点

```cpp
// 1. FK + 所有运动学量（一次性计算，由外层 computeRegressorMatrix 调用）
//    此处假设调用前已执行 forwardKinematics(model_, data_, q, qd, qdd)

// 2. 获取 body 局部系的 J 和 dJ/dt（替代 line 364-397）
pinocchio::Data::Matrix6x J_local(6, N_DOF);
getFrameJacobian(model_, data_, body_frame_ids_[body_idx], pinocchio::LOCAL, J_local);

pinocchio::Data::Matrix6x dJ_local(6, N_DOF);
getFrameJacobianTimeVariation(model_, data_, body_frame_ids_[body_idx],
                               pinocchio::LOCAL, dJ_local);

// 3. 获取局部系运动学量（替代 line 404-418）
Motion vel  = getFrameVelocity(model_, data_, body_frame_ids_[body_idx], pinocchio::LOCAL);
Motion acc  = getFrameAcceleration(model_, data_, body_frame_ids_[body_idx], pinocchio::LOCAL);

Vector3d omega_local = vel.angular();
Vector3d a_local     = acc.linear();
Vector3d alpha_local = acc.angular();

// 4. 分块 J（替代 line 399-402, 417-418）
auto Jv_local = J_local.topRows<3>();
auto Jw_local = J_local.bottomRows<3>();

// 5. 重力（替代 line 414）
Vector3d g_local = data_.oMf[body_frame_ids_[body_idx]].rotation().transpose() * gravity_;
Vector3d b_local = a_local - g_local;

// 6. 回归块构建 ———— 完全不变（line 420-453）
```

#### `computeRegressorMatrix()` 优化

```cpp
// 关键改变：只调用一次 forwardKinematics，所有 body 复用 data_
MatrixXd computeRegressorMatrix(q, qd, qdd, flags) {
  pinocchio::forwardKinematics(model_, data_, q, qd, qdd);  // 一次性计算

  MatrixXd Y = MatrixXd::Zero(N_DOF, num_params);
  for (size_t i = 0; i < N_BODIES; ++i) {
    // 每个 body 直接从 data_ 提取 J/dJ/acc（不再重复 FK）
    Y.block(0, i*10, N_DOF, 10) = computeBodyRegressorBlock(i + KINEMATIC_PREFIX, q, qd, qdd);
  }
  // armature / damping 不变
}
```

**预期加速**：FK 从 ~21 次/regressor → 1 次，dJ/dt 从数值差分 → 解析，整体 5-15x。

### 1.3 数值验证
- [ ] 编写 `test/validate_pinocchio_regressor.cpp`
  - 随机生成 100 组 `(q, qd, qdd)`
  - 同时用 `RevoarmNewRegressor` 和 `PinocchioRegressor` 计算 Y 矩阵
  - 逐元素比较：差异应 < 1e-6（解析 dJ/dt 理论上更精确）
  - 比较 torque 预测：用相同 θ 计算 τ = Y·θ，差异应 < 1e-4 Nm
- [ ] 运行全量辨识对比（OLS + 同一份数据），验证 RMSE 一致性

### 1.4 切换到新实现
- [ ] `main.cpp`: 将 `RevoarmNewRegressor` 替换为 `PinocchioRegressor`
- [ ] 旧的 `RevoarmNewRegressor` 保留作为 reference（不参与编译，仅备查）
- [ ] 确认所有现有算法（OLS..NLS_FRICTION）正常工作

---

## Phase 2 — YAML 驱动的机器人配置系统

**目标**：新机器人只需一个 YAML 文件即可完成辨识，无需编写 C++ 代码。

**参考形式**：Isaac Sim / Isaac Lab 的 robot description 模式。

### 2.1 YAML Schema 设计

#### 机器人描述文件 `robots/<robot_name>.yaml`

```yaml
# 机器人运动学与惯性参数描述 (自定义格式)
robot:
  name: "revoarm_right"
  dof: 7
  gravity: [0, 0, -9.81]

# 运动学链（按 tree 顺序排列，每行一个 link）
links:
  - name: "base_link"
    joint: fixed
    # placement 默认 identity

  - name: "right_base_link"
    joint: fixed
    placement:
      translation: [0.0055, -0.10424, -0.038037]
      quaternion: [0.9961948, -0.0871543, 0.0, 0.0]  # w,x,y,z

  - name: "right_arm_link1"
    joint:
      type: revolute
      axis: [0, 1, 0]        # 在 placement 坐标系中的旋转轴
    placement:                # joint 在父连杆中的位姿
      translation: [0, 0, 0]
      quaternion: [1, 0, 0, 0]
    inertial:
      mass: 0.99862
      com: [0.0013826, -0.0527123, -0.0012165]       # 质心在 body 系中的位置
      inertia:                                         # 绕质心的转动惯量
        Ixx: 0.0013454
        Iyy: 0.0008341
        Izz: 0.0010334
        Ixy: 9.3574e-06
        Ixz: 3.4782e-07
        Iyz: 4.9037e-05
    damping_prior: 0.1799     # 辨识先验 / 初始值
    armature_prior: 0.0

  # ... link2 ~ link7 同理
```

**设计要点**：
- `placement`: 固定 transform（joint 在父连杆 frame 中的位姿）→ 直接映射为 `pinocchio::SE3`
- `joint.axis`: 旋转轴方向 → 直接映射为 `pinocchio::JointModelRevolute(axis)`
- `inertial`: mass + COM + I_com → 直接映射为 `pinocchio::Inertia`
- 不需要 `has_joint` 字段——`joint: fixed` 即无关节的 kinematic link
- 不需要 `pos`/`quat` 分离——`placement` 统一表达

#### 辨识任务配置文件（扩展 `config/identification.yaml`）

```yaml
# 辨识任务配置
robot:
  config: "robots/revoarm_right.yaml"   # 机器人描述文件路径
  # 替代原来的 robot: "revoarm_new" 字符串选择

data:
  file: "data/revoarm_filtered_data_condnum_56.12_0618.csv"
  format: csv                          # 预留: csv | rosbag | mcap

algorithm:
  name: "OLS"                          # OLS | WLS | IRLS | TLS | EKF | ML | NLS_FRICTION | benchmark
  regularization: true
  flags: ["armature", "damping"]       # 辨识的参数类型

output:
  file: "result/identification.yaml"
  format: yaml                         # 预留: yaml | json | csv
```

### 2.2 实现

- [ ] **新建** `include/robot_config.hpp` / `src/robot_config.cpp`
  - YAML → `RobotConfig` 结构体（link 列表、惯性参数、先验值）
  - 从 `RobotConfig` 构建 `pinocchio::Model`
- [ ] **重构** `PinocchioRegressor` 构造函数
  - 接受 `RobotConfig` 而非硬编码 `initBodies()`
  - `initBodies()` 变为私有辅助，仅用于从 `RobotConfig` 填充 `bodies_` 数组
- [ ] **新建** `include/regressor_factory.hpp` / `src/regressor_factory.cpp`
  - `createRegressor(yaml_path)` → `std::unique_ptr<PinocchioRegressor>`
  - 封装 YAML 解析 + Model 构建 + regressor 实例化
- [ ] **重构** `main.cpp`
  - 用 `RegressorFactory` 替代硬编码 `RevoarmNewRegressor`
  - 配置文件中的 `robot.config` 路径决定加载哪个机器人
- [ ] **依赖**：项目已使用 yaml-cpp（或已在系统中），确认 CMakeLists 有 `find_package(yaml-cpp)`

### 2.3 新增机器人流程（最终效果）

```bash
# 1. 编写机器人描述文件
vim robots/my_robot.yaml

# 2. 修改辨识任务配置，指向新机器人
# robot.config: "robots/my_robot.yaml"

# 3. 运行
./identify --config config/identification.yaml
```

**零 C++ 代码改动。**

### 2.4 验证
- [ ] 用 `revoarm_right.yaml` 辨识，结果与 Phase 1 硬编码版一致
- [ ] 用不同构型机器人（如 Panda、Piper）的 YAML 测试，确认工厂正确构建模型
- [ ] `getParameterNames()` 自动从 YAML link 名称生成有意义的参数名

---

## Phase 3 — QT 上位机

**目标**：图形化操作，无需命令行和手动编辑 YAML，实现「加载数据 → 选算法 → 点按钮 → 出结果」的无脑辨识。

### 3.1 功能模块

| 模块 | 功能 |
|------|------|
| **机器人管理** | 从 YAML 加载机器人列表，可视化运动学树，编辑/导入新机器人 |
| **数据管理** | 拖入 CSV / 选择文件，预览关节轨迹曲线（q, qd, τ），自动检测 DOF |
| **算法配置** | 勾选/下拉选择算法（OLS..NLS_FRICTION），调节正则化参数 |
| **辨识执行** | 一键运行，实时显示进度，展示残差收敛曲线 |
| **结果展示** | 表格展示辨识出的惯性参数，bar chart 对比各 joint RMSE，torque 预测 vs 实测 overlay plot |
| **导出** | 参数导出 YAML/CSV，报告导出 PDF/HTML |

### 3.2 技术方案

| 选项 | 推荐 | 理由 |
|------|------|------|
| QT 版本 | **QT 6.x** | 长期支持，charts 模块成熟 |
| 图表库 | **QCharts** (QT 内置) | 免额外依赖，曲线/柱状图足够用 |
| C++ 绑定 | **直接调用** Phase 2 的 C++ 库 | 无中间层，性能最优 |
| 辨识线程 | **QThread / std::thread** | 辨识计算放入 worker 线程，UI 不阻塞 |
| YAML 编辑 | 内嵌文本编辑器 + 语法高亮 | 高级用户可直接编辑 YAML |

### 3.3 界面结构

```
┌──────────────────────────────────────────────────────┐
│  Menu: File | Robot | Help                           │
├────────────┬─────────────────────────────────────────┤
│  Robot     │                                         │
│  ────────  │   Torque Prediction vs Measured         │
│  revoarm   │   ┌─────────────────────────────────┐   │
│  panda     │   │  ═══ measured  ─── predicted     │   │
│  piper     │   │  ╱╲   ╱╲    ╱╲                 │   │
│            │   │ ╱  ╲ ╱  ╲  ╱  ╲                │   │
│  Data      │   │─────────────────────────────────│   │
│  ────────  │   │  time →                          │   │
│  browse..  │   └─────────────────────────────────┘   │
│  N=5000    │                                         │
│            │   Parameter Table                        │
│  Algorithm │   ┌──────────┬──────────┬──────────┐    │
│  ────────  │   │ Body     │ Mass     │ Ixx      │    │
│  [✓] OLS  │   │ link1    │ 0.9986   │ 0.0013   │    │
│  [ ] WLS  │   │ link2    │ 0.1942   │ 0.0004   │    │
│  [ ] IRLS │   │ ...      │ ...      │ ...      │    │
│            │   └──────────┴──────────┴──────────┘    │
│  ┌──────┐  │                                         │
│  │ RUN  │  │   RMSE: 0.0234 Nm                       │
│  └──────┘  │                                         │
├────────────┴─────────────────────────────────────────┤
│  Status: Ready                                        │
└──────────────────────────────────────────────────────┘
```

### 3.4 分步实现

- [ ] **3.4.1 项目搭建**：CMake QT 项目，链接 `libidentification_core`
- [ ] **3.4.2 机器人管理页**：YAML 列表加载 + 运动学树可视化
- [ ] **3.4.3 数据加载与预览页**：CSV 导入 → 关节轨迹 plot
- [ ] **3.4.4 辨识配置与执行页**：算法选择 + flags + 运行按钮
- [ ] **3.4.5 结果展示页**：参数表 + RMSE bar chart + torque overlay
- [ ] **3.4.6 导出功能**：YAML / CSV 参数导出
- [ ] **3.4.7 打包发布**：AppImage / 静态链接，确保无系统依赖

---

## 依赖与风险

| 依赖 | 当前状态 | 说明 |
|------|----------|------|
| Eigen 3 | ✅ 已集成 | 不变 |
| Pinocchio ≥ 2.6 | ❌ 需安装 | Phase 1 新增；`apt install ros-<dist>-pinocchio` 或源码编译 |
| yaml-cpp | ❌ 需集成 | Phase 2 新增（当前仅有手写 parser） |
| QT 6 | ❌ 需安装 | Phase 3 新增 |

| 风险 | 缓解 |
|------|------|
| Pinocchio Frame Jacobian 与手动 J 的坐标系约定不完全一致 | Phase 1 数值对比验证，必要时调整参考系参数 |
| 不同机器人的摩擦/armature 先验获取困难 | YAML 中 `damping_prior` / `armature_prior` 设为可选字段，默认 0 |
| QT 开发周期长 | 先完成 Phase 1+2 确保核心管线稳定，GUI 仅调用已有的 C++ 接口 |

---

## 里程碑

```
Phase 0  代码清理                 预计 0.5天
Phase 1  Pinocchio 替换 J         预计 2-3天
Phase 2  YAML 驱动配置            预计 2-3天
Phase 3  QT 上位机                 预计 5-7天
────────────────────────────────────────────
总计                               预计 10-14天
```
