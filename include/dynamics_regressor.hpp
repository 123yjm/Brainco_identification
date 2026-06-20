#pragma once

#include "body2inertial.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

namespace robot_dynamics {

/// 动力学回归器抽象基类
/// 具体机器人（如 SerialArmRegressor）继承此类，工厂按 robot 名创建实例
class IDynamicsRegressor {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;

  virtual ~IDynamicsRegressor() = default;

  /// 自由度数
  virtual std::size_t nDof() const = 0;

  /// 用于辨识的刚体数量（不含 kinematic prefix）
  virtual std::size_t nBodies() const = 0;

  /// 给定 flags 下的参数总数
  virtual std::size_t numParameters(ParamFlags flags = ParamFlags::ALL) const = 0;

  /// 从运动学配置计算名义参数向量
  virtual VectorXd computeParameterVector(ParamFlags flags = ParamFlags::ALL) const = 0;

  /// 参数名列表（用于结果展示）
  virtual std::vector<std::string> getParameterNames(ParamFlags flags = ParamFlags::ALL) const = 0;

  /// 单帧回归矩阵 Y (n_dof × n_params)
  virtual MatrixXd computeRegressorMatrix(
      const VectorXd &q, const VectorXd &qd, const VectorXd &qdd,
      ParamFlags flags = ParamFlags::ALL) const = 0;

  /// 堆叠观测矩阵 W (n_dof*K × n_params)
  virtual MatrixXd computeObservationMatrix(
      const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
      ParamFlags flags = ParamFlags::ALL) const = 0;
};

} // namespace robot_dynamics
