#pragma once

#include "dynamics_regressor.hpp"

#include <string>
#include <vector>

namespace robot_dynamics {

class SerialArmRegressor : public IDynamicsRegressor {
public:
  using Matrix3d = Eigen::Matrix3d;
  using Vector3d = Eigen::Vector3d;
  using Matrix4d = Eigen::Matrix4d;
  using Quaterniond = Eigen::Quaterniond;

  /// 从 YAML 配置文件构造（唯一入口）
  explicit SerialArmRegressor(const std::string &yaml_path);

  /// 自由度数
  std::size_t nDof() const override { return n_dof_; }

  /// 用于辨识的刚体数量（不含 kinematic prefix）
  std::size_t nBodies() const override { return n_bodies_; }

  VectorXd
  computeParameterVector(ParamFlags flags = ParamFlags::ALL) const override;

  std::size_t
  numParameters(ParamFlags flags = ParamFlags::ALL) const override;

  MatrixXd
  computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                         const VectorXd &qdd,
                         ParamFlags flags = ParamFlags::ALL) const override;

  MatrixXd computeObservationMatrix(
      const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
      ParamFlags flags = ParamFlags::ALL) const override;

  std::vector<std::string>
  getParameterNames(ParamFlags flags = ParamFlags::ALL) const override;

private:
  std::size_t n_dof_ = 0;
  std::size_t n_bodies_ = 0;
  std::size_t kinematic_prefix_ = 0;

  /// 全部刚体，索引 0..kinematic_prefix_-1 为纯运动学刚体，
  /// kinematic_prefix_.. 为用于辨识的关节刚体
  std::vector<RigidBody> bodies_;
  Vector3d gravity_{0, 0, -9.81};

  void loadFromYaml(const std::string &path);

  std::vector<Matrix4d> computeBodyTransforms(const VectorXd &q) const;
  Vector3d computeBodyCOM(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeBodyOriginJacobian(std::size_t body_idx,
                                     const VectorXd &q) const;
  MatrixXd computeBodyOriginJacobianDerivative(std::size_t body_idx,
                                               const VectorXd &q,
                                               const VectorXd &qd) const;
  MatrixXd computeBodyJacobian(std::size_t body_idx, const VectorXd &q) const;
  MatrixXd computeBodyJacobianDerivative(std::size_t body_idx,
                                         const VectorXd &q,
                                         const VectorXd &qd) const;

  static Matrix3d skew(const Vector3d &v);
  static Matrix4d poseToTransform(const Vector3d &pos, const Quaterniond &quat);

  MatrixXd computeBodyRegressorBlock(std::size_t body_idx,
                                     const std::vector<Matrix4d> &transforms,
                                     const std::vector<Matrix4d> &transforms_plus,
                                     const VectorXd &qd,
                                     const VectorXd &qdd) const;
};

} // namespace robot_dynamics
