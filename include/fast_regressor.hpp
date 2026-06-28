#pragma once

#include "dynamics_regressor.hpp"

#include <Eigen/Core>

namespace robot_dynamics {

class SerialArmRegressor;

/// 初始化快速回归器缓存（从 SerialArmRegressor 提取常量参数）
void initFastRegressor(const SerialArmRegressor &reg);

/// 快速计算单帧回归矩阵 Y (n_dof × n_params)
/// 使用 sin/cos + 几何 Jacobian，无 FK/四元数/数值微分
Eigen::MatrixXd computeRegressorFast(const Eigen::VectorXd &q,
                                      const Eigen::VectorXd &qd,
                                      const Eigen::VectorXd &qdd,
                                      ParamFlags flags = ParamFlags::ALL);

/// 快速堆叠观测矩阵 W (n_dof*K × n_params)
Eigen::MatrixXd computeObservationMatrixFast(const Eigen::MatrixXd &Q,
                                              const Eigen::MatrixXd &Qd,
                                              const Eigen::MatrixXd &Qdd,
                                              ParamFlags flags = ParamFlags::ALL);

} // namespace robot_dynamics
