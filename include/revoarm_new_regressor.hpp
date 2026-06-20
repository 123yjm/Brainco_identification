#pragma once

#include "body2inertial.hpp"

namespace robot_dynamics {

class RevoarmNewRegressor {
public:
  static constexpr std::size_t N_DOF = 7;
  // 带质量用于辨识的 body: link1-7 = 7
  // (link7 已合并 connector_link + hand_base_link 的质量和惯量)
  static constexpr std::size_t N_BODIES = 7;

  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
  using Matrix3d = Eigen::Matrix3d;
  using Vector3d = Eigen::Vector3d;
  using Matrix4d = Eigen::Matrix4d;
  using Quaterniond = Eigen::Quaterniond;

  RevoarmNewRegressor();

  VectorXd
  computeParameterVector(ParamFlags flags = ParamFlags::ALL) const;

  std::size_t
  numParameters(ParamFlags flags = ParamFlags::ALL) const;

  MatrixXd
  computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                         const VectorXd &qdd,
                         ParamFlags flags = ParamFlags::ALL) const;

  MatrixXd computeObservationMatrix(
      const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
      ParamFlags flags = ParamFlags::ALL) const;

  std::vector<std::string>
  getParameterNames(ParamFlags flags = ParamFlags::ALL) const;

private:
  // 前方放 base_link + right_base_link (纯运动学, 不计入 N_BODIES)
  // 索引: [0]=base_link, [1]=right_base_link, [2..8]=7 个 regressor body
  std::array<RigidBody, N_BODIES + 2> bodies_;
  Vector3d gravity_{0, 0, -9.81};

  void initBodies();

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

  MatrixXd computeBodyRegressorBlock(std::size_t body_idx, const VectorXd &q,
                                     const VectorXd &qd,
                                     const VectorXd &qdd) const;
};

} // namespace robot_dynamics
