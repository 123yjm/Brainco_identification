#include "revoarm_new_regressor.hpp"
#include "robot_config_loader.hpp"

#include <iostream>

namespace robot_dynamics {

// ============================================================================
// 构造函数 — 从 YAML 加载
// ============================================================================

RevoarmNewRegressor::RevoarmNewRegressor(const std::string &yaml_path) {
  loadFromYaml(yaml_path);
}

void RevoarmNewRegressor::loadFromYaml(const std::string &path) {
  RobotConfig cfg = loadKinematicConfig(path);

  n_dof_ = cfg.dof;
  kinematic_prefix_ = cfg.kinematic_prefix;

  // 辨识刚体数量 = 总刚体 - kinematic prefix
  if (cfg.bodies.size() <= kinematic_prefix_) {
    throw std::runtime_error(
        "bodies 数量 (" + std::to_string(cfg.bodies.size()) +
        ") 不大于 kinematic_prefix (" + std::to_string(kinematic_prefix_) + ")");
  }
  n_bodies_ = cfg.bodies.size() - kinematic_prefix_;

  bodies_ = std::move(cfg.bodies);

  std::cout << "[RevoarmNewRegressor] 从 " << path << " 加载, "
            << n_dof_ << " DOF, " << n_bodies_ << " regressor bodies, "
            << kinematic_prefix_ << " kinematic prefix" << std::endl;
}

// ============================================================================
// 辅助函数
// ============================================================================

RevoarmNewRegressor::Matrix3d
RevoarmNewRegressor::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

RevoarmNewRegressor::Matrix4d
RevoarmNewRegressor::poseToTransform(const Vector3d &pos,
                                            const Quaterniond &quat) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

// ============================================================================
// 运动学
// ============================================================================

std::vector<RevoarmNewRegressor::Matrix4d>
RevoarmNewRegressor::computeBodyTransforms(const VectorXd &q) const {
  const std::size_t total = kinematic_prefix_ + n_bodies_;
  std::vector<Matrix4d> transforms(total);

  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  std::size_t joint_idx = 0;

  for (std::size_t i = 1; i < total; ++i) {
    const auto &body = bodies_[i];
    Matrix4d T_parent_body = poseToTransform(body.pos, body.quat);

    if (body.has_joint && joint_idx < n_dof_) {
      double angle = q(joint_idx);
      Quaterniond joint_rot =
          Quaterniond(Eigen::AngleAxisd(angle, body.joint_axis));
      Matrix4d T_joint = Matrix4d::Identity();
      T_joint.block<3, 3>(0, 0) = joint_rot.toRotationMatrix();
      transforms[i] = transforms[i - 1] * T_parent_body * T_joint;
      ++joint_idx;
    } else {
      transforms[i] = transforms[i - 1] * T_parent_body;
    }
  }

  return transforms;
}

RevoarmNewRegressor::Vector3d
RevoarmNewRegressor::computeBodyCOM(std::size_t body_idx,
                                           const VectorXd &q) const {
  auto transforms = computeBodyTransforms(q);
  Vector3d com_local = bodies_[body_idx].com;
  Matrix4d T = transforms[body_idx];
  return T.block<3, 3>(0, 0) * com_local + T.block<3, 1>(0, 3);
}

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeBodyOriginJacobian(
    std::size_t body_idx, const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, n_dof_);
  auto transforms = computeBodyTransforms(q);
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3);

  const std::size_t total = kinematic_prefix_ + n_bodies_;
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i < total; ++i) {
    if (bodies_[i].has_joint) {
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_origin - p_joint);
      J.block<3, 1>(3, joint_count) = z_axis;
      ++joint_count;
    }
  }

  return J;
}

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeBodyJacobian(std::size_t body_idx,
                                                const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, n_dof_);
  auto transforms = computeBodyTransforms(q);
  Vector3d p_body = computeBodyCOM(body_idx, q);

  const std::size_t total = kinematic_prefix_ + n_bodies_;
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i < total; ++i) {
    if (bodies_[i].has_joint) {
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_body - p_joint);
      J.block<3, 1>(3, joint_count) = z_axis;
      ++joint_count;
    }
  }

  return J;
}

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeBodyOriginJacobianDerivative(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd) const {
  const double eps = 1e-7;
  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;
  MatrixXd J_plus = computeBodyOriginJacobian(body_idx, q_plus);
  MatrixXd J_minus = computeBodyOriginJacobian(body_idx, q_minus);
  return (J_plus - J_minus) / (2.0 * eps);
}

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeBodyJacobianDerivative(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd) const {
  const double eps = 1e-7;
  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;
  MatrixXd J_plus = computeBodyJacobian(body_idx, q_plus);
  MatrixXd J_minus = computeBodyJacobian(body_idx, q_minus);
  return (J_plus - J_minus) / (2.0 * eps);
}

// ============================================================================
// 参数向量
// ============================================================================

std::size_t
RevoarmNewRegressor::numParameters(ParamFlags flags) const {
  std::size_t params = n_bodies_ * InertialParams::PARAMS_PER_BODY;

  if (hasFlag(flags, ParamFlags::DAMPING)) {
    params += n_dof_;
  }

  return params;
}

RevoarmNewRegressor::VectorXd
RevoarmNewRegressor::computeParameterVector(ParamFlags flags) const {
  const std::size_t num_params = numParameters(flags);
  VectorXd theta = VectorXd::Zero(num_params);

  for (std::size_t i = 0; i < n_bodies_; ++i) {
    std::size_t body_idx = i + kinematic_prefix_;
    auto sip = InertialParams::fromBody(bodies_[body_idx]);
    auto sip_vec = sip.toVector();
    std::size_t offset = i * InertialParams::PARAMS_PER_BODY;
    theta.segment(offset, InertialParams::PARAMS_PER_BODY) = sip_vec;
  }

  // damping 无物理先验，填 0
  if (hasFlag(flags, ParamFlags::DAMPING)) {
    // theta 已初始化为零，无需额外操作
  }

  return theta;
}

std::vector<std::string>
RevoarmNewRegressor::getParameterNames(ParamFlags flags) const {
  std::vector<std::string> names;

  const char *param_names[] = {"m",   "mx",  "my",  "mz",  "Ixx",
                               "Ixy", "Ixz", "Iyy", "Iyz", "Izz"};

  for (std::size_t i = 0; i < n_bodies_; ++i) {
    std::size_t body_idx = i + kinematic_prefix_;
    for (int j = 0; j < 10; ++j) {
      names.push_back(bodies_[body_idx].name + "_" + param_names[j]);
    }
  }

  if (hasFlag(flags, ParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < n_dof_; ++i) {
      names.push_back("damping_" + std::to_string(i + 1));
    }
  }

  return names;
}

// ============================================================================
// 回归矩阵
// ============================================================================

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeBodyRegressorBlock(
    std::size_t body_idx,
    const std::vector<Matrix4d> &transforms,
    const std::vector<Matrix4d> &transforms_plus,
    const VectorXd &qd,
    const VectorXd &qdd) const {

  (void)qd; // qd 已隐含在 transforms_plus 的差分中

  Matrix3d R = transforms[body_idx].block<3, 3>(0, 0);
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3);

  // Body 原点的雅可比 (世界坐标系)
  MatrixXd J_world = MatrixXd::Zero(6, n_dof_);
  const std::size_t total = kinematic_prefix_ + n_bodies_;
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i < total; ++i) {
    if (bodies_[i].has_joint) {
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);
      J_world.block<3, 1>(0, joint_count) = z_axis.cross(p_origin - p_joint);
      J_world.block<3, 1>(3, joint_count) = z_axis;
      ++joint_count;
    }
  }

  // 雅可比导数（数值微分，复用外部传入的 transforms_plus）
  Vector3d p_origin_plus = transforms_plus[body_idx].block<3, 1>(0, 3);

  MatrixXd J_world_plus = MatrixXd::Zero(6, n_dof_);
  joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i < total; ++i) {
    if (bodies_[i].has_joint) {
      Vector3d z_axis =
          transforms_plus[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
      Vector3d p_joint = transforms_plus[i].block<3, 1>(0, 3);
      J_world_plus.block<3, 1>(0, joint_count) =
          z_axis.cross(p_origin_plus - p_joint);
      J_world_plus.block<3, 1>(3, joint_count) = z_axis;
      ++joint_count;
    }
  }
  const double dt = 1e-7;
  MatrixXd J_world_dot = (J_world_plus - J_world) / dt;

  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_world = J_world.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_world = J_world.bottomRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_dot_world = J_world_dot.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_dot_world = J_world_dot.bottomRows(3);

  // 世界坐标系中的运动学量
  Vector3d a_origin_world = Jv_world * qdd + Jv_dot_world * qd;
  Vector3d omega_world = Jw_world * qd;
  Vector3d alpha_world = Jw_world * qdd + Jw_dot_world * qd;

  // 转换到 Body 局部坐标系
  Matrix3d Rt = R.transpose();
  Vector3d a_local = Rt * a_origin_world;
  Vector3d omega_local = Rt * omega_world;
  Vector3d alpha_local = Rt * alpha_world;
  Vector3d g_local = Rt * gravity_;
  Vector3d b_local = a_local - g_local;

  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_local = Rt * Jv_world;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_local = Rt * Jw_world;

  // 构建回归矩阵块 Y_body (n_dof_ × 10)
  MatrixXd Y_block = MatrixXd::Zero(n_dof_, 10);

  // K = [α]× + [ω]× * [ω]×
  Matrix3d alpha_skew = skew(alpha_local);
  Matrix3d omega_skew = skew(omega_local);
  Matrix3d K = alpha_skew + omega_skew * omega_skew;

  double ox = omega_local(0), oy = omega_local(1), oz = omega_local(2);
  double ax = alpha_local(0), ay = alpha_local(1), az = alpha_local(2);

  // 1. 质量 m
  Y_block.col(0) = Jv_local.transpose() * b_local;

  // 2. 一阶矩 (mx, my, mz)
  for (int i = 0; i < 3; ++i) {
    Vector3d e_i = Vector3d::Zero();
    e_i(i) = 1.0;
    Y_block.col(1 + i) =
        Jv_local.transpose() * K.col(i) + Jw_local.transpose() * e_i.cross(b_local);
  }

  // 3. 惯量张量
  Y_block.col(4) = Jw_local.transpose() * Vector3d(ax, ox * oz, -ox * oy);
  Y_block.col(5) = Jw_local.transpose() *
                    Vector3d(ay - ox * oz, ax + oy * oz, ox * ox - oy * oy);
  Y_block.col(6) = Jw_local.transpose() *
                    Vector3d(az + ox * oy, oz * oz - ox * ox, ax - oy * oz);
  Y_block.col(7) = Jw_local.transpose() * Vector3d(-oy * oz, ay, ox * oy);
  Y_block.col(8) = Jw_local.transpose() *
                    Vector3d(oy * oy - oz * oz, az - ox * oy, ay + ox * oz);
  Y_block.col(9) = Jw_local.transpose() * Vector3d(oy * oz, -ox * oz, az);

  return Y_block;
}

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeRegressorMatrix(
    const VectorXd &q, const VectorXd &qd, const VectorXd &qdd,
    ParamFlags flags) const {

  const std::size_t num_params = numParameters(flags);
  MatrixXd Y = MatrixXd::Zero(n_dof_, num_params);

  // 一次 FK，全部刚体共享（而非每个 body 重复计算）
  auto transforms = computeBodyTransforms(q);
  const double dt = 1e-7;
  auto transforms_plus = computeBodyTransforms(q + dt * qd);

  for (std::size_t i = 0; i < n_bodies_; ++i) {
    std::size_t body_idx = i + kinematic_prefix_;
    MatrixXd Y_body = computeBodyRegressorBlock(body_idx, transforms,
                                                 transforms_plus, qd, qdd);
    std::size_t offset = i * InertialParams::PARAMS_PER_BODY;
    Y.block(0, offset, n_dof_, InertialParams::PARAMS_PER_BODY) = Y_body;
  }

  if (hasFlag(flags, ParamFlags::DAMPING)) {
    std::size_t current_offset = n_bodies_ * InertialParams::PARAMS_PER_BODY;
    for (std::size_t i = 0; i < n_dof_; ++i) {
      Y(i, current_offset + i) = -qd(i);
    }
  }

  return Y;
}

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeObservationMatrix(
    const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
    ParamFlags flags) const {

  const std::size_t K = Q.cols();
  const std::size_t num_params = numParameters(flags);
  MatrixXd W = MatrixXd::Zero(n_dof_ * K, num_params);

#ifdef IDENTIFICATION_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(K); ++k) {
    VectorXd q = Q.col(k);
    VectorXd qd = Qd.col(k);
    VectorXd qdd = Qdd.col(k);
    MatrixXd Y_k = computeRegressorMatrix(q, qd, qdd, flags);
    W.block(k * static_cast<Eigen::Index>(n_dof_), 0,
            static_cast<Eigen::Index>(n_dof_),
            static_cast<Eigen::Index>(num_params)) = Y_k;
  }

  return W;
}

} // namespace robot_dynamics
