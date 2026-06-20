# Plan: Pinocchio 替换手动雅可比计算

## Context

当前 `RevoarmNewRegressor::computeBodyRegressorBlock` (以及 revoarm_right 变体) 中，雅可比矩阵 `J` 及其时间导数 `dJ/dt` 完全通过**手动几何方法 + 数值微分**计算：

- **FK**: 手动四元数链式乘法 (`computeBodyTransforms`)
- **J**: 逐列构建 `z × (p_origin - p_joint)` + `z` (世界坐标系)
- **dJ/dt**: 前向差分 `(J(q+ε·qd) - J(q)) / ε`，ε=1e-7

这种方式存在以下问题：
1. **数值噪声**：有限差分放大了浮点误差，尤其在高加速度时
2. **计算冗余**：每个 body 重复计算 FK 和 J
3. **维护成本高**：切换机器人需手动重写所有运动学参数
4. **无分析性 dJ/dt**：Pinocchio 提供 RNEA 衍生的解析 dJ/dt

目标：将雅可比及运动学量的计算**迁移到 Pinocchio**，保留回归矩阵构建逻辑不变。

## Current Implementation Breakdown

### 1. 前向运动学 (`computeBodyTransforms`, :166-193)
```
T[0] = I
for i=1..8:
  T_parent_body = pose_to_transform(body.pos, body.quat)
  if has_joint:
    T_joint = AngleAxis(q[joint_idx], joint_axis)
    T[i] = T[i-1] * T_parent_body * T_joint
  else:
    T[i] = T[i-1] * T_parent_body
```

### 2. 雅可比 (`computeBodyRegressorBlock` 内嵌, :364-397)
对 body `body_idx`，遍历从 1 到 body_idx 的所有 body：
- 有 joint → 计算 z_axis (世界系) = R_i · joint_axis
- 线性部分: `z × (p_origin - p_joint)`
- 角速度部分: `z`
- dJ/dt 通过前向差分: `(J(q+dt*qd) - J(q)) / dt`

### 3. 运动学量 (:399-418)
```
a_world   = Jv·qdd + Jv_dot·qd
ω_world   = Jw·qd
α_world   = Jw·qdd + Jw_dot·qd
→ 通过 Rᵀ 变换到 body 局部坐标系
b_local   = a_local - g_local
```

### 4. 回归块 (:420-453)
使用 `Jv_local`, `Jw_local`, `ω_local`, `α_local`, `b_local` 构建 Y_block (N_DOF × 10)

## Pinocchio Approach

### 核心 API 映射

| 当前手动方法 | Pinocchio 等价 |
|---|---|
| `computeBodyTransforms(q)` | `pinocchio::forwardKinematics(model, data, q)` |
| 手动构建 J_world | `pinocchio::computeJointJacobians(model, data, q)` + `getJointJacobian` |
| 数值 dJ/dt | `pinocchio::computeJointJacobiansTimeVariation(model, data, q, qd)` |
| Body 原点雅可比 | `pinocchio::getFrameJacobian(model, data, frame_id, LOCAL/WORLD)` |
| Body 加速度 | `pinocchio::getFrameAcceleration(model, data, frame_id, LOCAL/WORLD)` |
| `Rᵀ * a_world` | 直接用 `LOCAL` 参考系，Pinocchio 自动处理 |

### 关键改进

1. **Frame Jacobian**: 为每个 regressor body 在 Pinocchio Model 中定义 frame (位于 body origin)，直接用 `getFrameJacobian(model, data, frame_id, pinocchio::LOCAL)` 获取 body 局部坐标系下的雅可比——**省去手动 Rᵀ 变换**

2. **解析 dJ/dt**: `pinocchio::getFrameJacobianTimeVariation(model, data, frame_id, pinocchio::LOCAL)` 直接给出分析性雅可比时间导数

3. **Frame 加速度**: `pinocchio::getFrameAcceleration(model, data, frame_id, pinocchio::LOCAL)` 直接给出 body 局部坐标系下的 `a_local` 和 `α_local`

4. **统一模型**: Pinocchio Model 从 URDF 构建，支持任意机器人，无需硬编码 `initBodies()`

## Implementation Plan

### Step 1: 添加 Pinocchio 依赖

**文件**: `CMakeLists.txt`

- 添加 `find_package(pinocchio REQUIRED)`
- 链接 `pinocchio::pinocchio` 到 `identification_core`

### Step 2: 构建 Pinocchio Model

**新建文件**: `src/identification/include/pinocchio_revoarm_new_regressor.hpp`
**新建文件**: `src/identification/src/robot/pinocchio_revoarm_new_regressor.cpp`

设计新的 `PinocchioRevoarmNewRegressor` 类：

```cpp
class PinocchioRevoarmNewRegressor {
public:
  static constexpr std::size_t N_DOF = 7;
  static constexpr std::size_t N_BODIES = 7;

  PinocchioRevoarmNewRegressor();

  // 保持与原接口一致
  VectorXd computeParameterVector(...) const;
  MatrixXd computeRegressorMatrix(q, qd, qdd, ...) const;
  MatrixXd computeObservationMatrix(Q, Qd, Qdd, ...) const;

private:
  pinocchio::Model model_;      // 从 URDF/MJCF 构建
  pinocchio::Data data_;        // Pinocchio 数据缓存
  std::vector<pinocchio::FrameIndex> body_frame_ids_;  // 每个 regressor body 的 frame ID
  Vector3d gravity_{0, 0, -9.81};

  // 新实现的核心方法
  MatrixXd computeBodyRegressorBlock(size_t body_idx, q, qd, qdd) const;
};
```

**Model 构建策略**：由于 Pinocchio 原生支持 URDF，需要：
- 从 `revoarm_right/revoarm_right.xml` (MJCF) 手动提取运动学链
- **或** 直接在代码中用 `pinocchio::Model::buildJoint` 等 API 构建
- **推荐**：使用 `pinocchio::urdf::buildModel` 从 URDF 加载；如果有可用的 URDF，直接用；否则用 Pinocchio 的 C++ API 手动构建 Model (因为现有 MJCF 无对应 URDF parser)

### Step 3: 重写 `computeBodyRegressorBlock` 用 Pinocchio

核心变化——**原 100 行的手动雅可比计算简化为 ~30 行**:

```cpp
MatrixXd computeBodyRegressorBlock(size_t body_idx, q, qd, qdd) {
  // 1. 前向运动学 (一次性计算所有量)
  pinocchio::forwardKinematics(model_, data_, q, qd, qdd);
  // 这一步同时计算了: 所有 frame 位置/速度/加速度

  // 2. 获取 body origin frame 的局部雅可比和 dJ/dt
  pinocchio::FrameIndex fid = body_frame_ids_[body_idx];
  
  // 局部坐标系下的雅可比 (6 × N_DOF)
  MatrixXd J_local(6, N_DOF);
  pinocchio::getFrameJacobian(model_, data_, fid, 
      pinocchio::LOCAL, J_local);
  
  // 局部坐标系下的 dJ/dt (6 × N_DOF)
  MatrixXd dJ_local(6, N_DOF);
  pinocchio::getFrameJacobianTimeVariation(model_, data_, fid,
      pinocchio::LOCAL, dJ_local);

  // 3. 局部坐标系运动学量 (Pinocchio 已经算好了)
  // getFrameAcceleration 返回 6D 运动旋量 (线加速度 + 角加速度)
  pinocchio::Motion acc = pinocchio::getFrameAcceleration(
      model_, data_, fid, pinocchio::LOCAL);
  Vector3d a_local = acc.linear();
  Vector3d alpha_local = acc.angular();
  
  // 角速度从 data 中获取
  pinocchio::Motion vel = pinocchio::getFrameVelocity(
      model_, data_, fid, pinocchio::LOCAL);
  Vector3d omega_local = vel.angular();

  // 4. 雅可比分块
  auto Jv_local = J_local.topRows<3>();
  auto Jw_local = J_local.bottomRows<3>();

  // 5. 重力
  Vector3d g_local = data_.oMf[fid].rotation().transpose() * gravity_;
  Vector3d b_local = a_local - g_local;

  // 6. 构建回归块 (保持不变)
  // ... 与原代码相同的 K, Y_block 构建逻辑 ...
}
```

### Step 4: 适配 `computeRegressorMatrix` 和 `computeObservationMatrix`

接口保持不变，内部调用新的 Pinocchio 版 `computeBodyRegressorBlock`。

**性能优化注意事项**：
- 原实现每个 body 都独立调用 `computeBodyTransforms(q)` (重复计算 FK)
- Pinocchio 版本中，`computeRegressorMatrix` 应**只调用一次** `forwardKinematics`，然后对每个 body_idx 复用 `data_`
- 需要在 `computeRegressorMatrix` 中重构调用方式：

```cpp
MatrixXd computeRegressorMatrix(q, qd, qdd, flags) {
  // 一次性计算所有运动学
  pinocchio::forwardKinematics(model_, data_, q, qd, qdd);
  
  MatrixXd Y = MatrixXd::Zero(N_DOF, num_params);
  for (size_t i = 0; i < N_BODIES; ++i) {
    // 复用 data_，不再重复 FK
    MatrixXd Y_body = computeBodyRegressorBlockFromData(i, q, qd, qdd);
    Y.block(0, i*10, N_DOF, 10) = Y_body;
  }
  // ... armature/damping 部分不变
}
```

### Step 5: 验证

#### 5a. 数值对比测试
- 编写测试：对随机 `(q, qd, qdd)`，同时用新旧 regressor 计算 Y 矩阵
- 比较 Y 矩阵的逐元素差异 (应 < 1e-6，因 Pinocchio 使用解析 dJ/dt)
- 比较 torque 预测差异 (使用相同参数向量 `theta`)

#### 5b. 参数辨识回归测试
- 使用相同的实验数据 (`revoarm_filtered_data_condnum_56.12_0616.csv`)
- 运行 OLS 辨识
- 比较辨识出的参数向量和 torque RMSE
- 预期：Pinocchio 版本的 RMSE 应**优于**当前版本（解析 dJ/dt 消除数值噪声）

### Files to Modify

| 文件 | 操作 |
|------|------|
| `CMakeLists.txt` | 添加 `find_package(pinocchio)` 和链接 |
| `src/identification/include/pinocchio_revoarm_new_regressor.hpp` | **新建** - Pinocchio 版 regressor 头文件 |
| `src/identification/src/robot/pinocchio_revoarm_new_regressor.cpp` | **新建** - Pinocchio 版 regressor 实现 |
| `src/identification/src/main.cpp` | 修改 - 添加 `"pinocchio_revoarm_new"` 机器人类型选项 |

### Files NOT Modified (作为参考保留)

| 文件 | 原因 |
|------|------|
| `revoarm_new_regressor.hpp/.cpp` | 保留手动实现作为 baseline/fallback |
| `revoarm_new_dynamics.hpp/.cpp` | 动力学验证暂不迁移 |
| `identification/algorithms.hpp/.cpp` | 算法层与运动学实现无关 |

## Verification

1. **编译**: `mkdir build && cd build && cmake .. && make` (需要系统已安装 pinocchio)
2. **单元测试**: 运行 `regressor_test` (如存在)，或编写简单的 main 对比新旧 Y 矩阵
3. **辨识管道**: 运行 `./identify --robot pinocchio_revoarm_new --data-file data/revoarm_filtered_data_condnum_56.12_0616.csv --output results/pinocchio_test.yaml`
4. **对比结果**: 比较 `results/pinocchio_test.yaml` 与 `results/identification_0616.yaml` 的 OLS torque_rmse
