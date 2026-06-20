#include "revoarm_new_regressor.hpp"
#include "robot_config_loader.hpp"

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/inertia.hpp>

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

  buildPinocchioModel();
  pin_data_ = pinocchio::Data(pin_model_);
}

// ============================================================================
// 构建 Pinocchio 模型
// ============================================================================

void RevoarmNewRegressor::buildPinocchioModel() {
  pin_model_ = pinocchio::Model();

  // 累积 kinematic prefix 的固定位姿
  pinocchio::SE3 prefix_placement = pinocchio::SE3::Identity();
  for (std::size_t i = 0; i < kinematic_prefix_; ++i) {
    prefix_placement = prefix_placement *
        pinocchio::SE3(bodies_[i].quat.toRotationMatrix(), bodies_[i].pos);
  }

  pinocchio::JointIndex parent = 0;  // universe 关节 (id=0)

  regressor_joint_ids_.clear();
  regressor_joint_ids_.reserve(n_bodies_);

  for (std::size_t i = 0; i < n_bodies_; ++i) {
    const auto &body = bodies_[kinematic_prefix_ + i];

    // 该 body 相对上一级的位姿
    pinocchio::SE3 placement(body.quat.toRotationMatrix(), body.pos);

    // 第一个辨识关节需要叠加 prefix
    if (i == 0) {
      placement = prefix_placement * placement;
    }

    pinocchio::JointModelRevoluteUnaligned jmodel(body.joint_axis);
    parent = pin_model_.addJoint(parent, jmodel, placement, body.name);
    regressor_joint_ids_.push_back(parent);

    // 附加惯量（质心坐标系下的惯量）
    pinocchio::Inertia inertia(
        body.mass,
        body.com,
        pinocchio::Symmetric3(body.Ixx, body.Iyy, body.Izz,
                              body.Ixy, body.Ixz, body.Iyz));
    pin_model_.appendBodyToJoint(parent, inertia, pinocchio::SE3::Identity());
  }

  std::cout << "[RevoarmNewRegressor] Pinocchio 模型构建完成, "
            << pin_model_.njoints << " joints, "
            << pin_model_.nq << " config dim" << std::endl;
}

// ============================================================================
// 参数向量（不受 Pinocchio 影响）
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

  // damping 无物理先验，填 0（theta 已初始化为零）
  (void)flags;
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
// 回归矩阵块 — 接收预计算的运动学量
// ============================================================================

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeBodyRegressorBlock(
    std::size_t body_idx,
    const MatrixXd &J_origin,
    const MatrixXd &J_origin_dot,
    const Matrix3d &R_body,
    const Vector3d &a_local,
    const Vector3d &omega_local,
    const Vector3d &alpha_local) const {

  (void)body_idx; // 保留参数，便于后续扩展

  // Jacobian 分块
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_world = J_origin.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_world = J_origin.bottomRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_dot_world = J_origin_dot.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_dot_world = J_origin_dot.bottomRows(3);

  // 转换到 Body 局部坐标系
  Matrix3d Rt = R_body.transpose();
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_local = Rt * Jv_world;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_local = Rt * Jw_world;

  // 重力补偿
  Vector3d g_local = Rt * gravity_;
  Vector3d b_local = a_local - g_local;

  // 构建回归矩阵块 Y_body (n_dof_ × 10)
  MatrixXd Y_block = MatrixXd::Zero(n_dof_, 10);

  // K = [α]× + [ω]× * [ω]×
  auto skew3 = [](const Vector3d &v) -> Matrix3d {
    Matrix3d S;
    S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
  };
  Matrix3d alpha_skew = skew3(alpha_local);
  Matrix3d omega_skew = skew3(omega_local);
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

// ============================================================================
// 回归矩阵 — Pinocchio 驱动的单次 FK + Jacobian
// ============================================================================

RevoarmNewRegressor::MatrixXd
RevoarmNewRegressor::computeRegressorMatrix(
    const VectorXd &q, const VectorXd &qd, const VectorXd &qdd,
    ParamFlags flags) const {

  const std::size_t num_params = numParameters(flags);
  MatrixXd Y = MatrixXd::Zero(n_dof_, num_params);

  // ---- 一次 FK + Jacobian + 时间导数 ----
  pinocchio::forwardKinematics(pin_model_, pin_data_, q, qd);
  pinocchio::computeJointJacobians(pin_model_, pin_data_, q);
  pinocchio::computeJointJacobiansTimeVariation(pin_model_, pin_data_, q, qd);

  // ---- 遍历辨识刚体 ----
  for (std::size_t i = 0; i < n_bodies_; ++i) {
    pinocchio::JointIndex jid = regressor_joint_ids_[i];

    // Jacobian 与导数（世界坐标系）
    MatrixXd J_origin(6, n_dof_);
    pinocchio::getJointJacobian(pin_model_, pin_data_, jid,
                                 pinocchio::WORLD, J_origin);
    MatrixXd J_origin_dot(6, n_dof_);
    pinocchio::getJointJacobianTimeVariation(pin_model_, pin_data_, jid,
                                               pinocchio::WORLD, J_origin_dot);

    // 位姿
    const pinocchio::SE3 &placement = pin_data_.oMi[jid];
    Matrix3d R_body = placement.rotation();
    Vector3d p_body = placement.translation();

    // Pinocchio WORLD Jacobian 在世界原点表达，需转换到 body 原点
    // J_body = [I  -skew(p_body); 0  I] * J_world
    auto skew3 = [](const Vector3d &v) -> Matrix3d {
      Matrix3d S;
      S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      return S;
    };
    MatrixXd X(6, 6);
    X.setZero();
    X.block<3, 3>(0, 0) = Matrix3d::Identity();
    X.block<3, 3>(0, 3) = -skew3(p_body);
    X.block<3, 3>(3, 3) = Matrix3d::Identity();
    MatrixXd J_body = X * J_origin;
    // J_body_dot = dX/dt * J_world + X * dJ_world
    // dX/dt = [0, -skew(v_body); 0, 0], v_body = J_body * qd
    MatrixXd J_body_dot = X * J_origin_dot;
    Vector3d v_body = J_body.topRows(3) * qd;
    J_body_dot.topRows(3) -= skew3(v_body) * J_origin.bottomRows(3);

    // 运动学量：用 J*qdd + dJ*qd 计算（与手写版本数值等价）
    VectorXd a_spatial_world = J_body * qdd + J_body_dot * qd;
    Vector3d a_origin_world = a_spatial_world.head(3);
    Vector3d omega_world = J_body.bottomRows(3) * qd;
    Vector3d alpha_world = a_spatial_world.tail(3);

    Matrix3d Rt = R_body.transpose();
    Vector3d a_local = Rt * a_origin_world;
    Vector3d omega_local = Rt * omega_world;
    Vector3d alpha_local = Rt * alpha_world;

    MatrixXd Y_body = computeBodyRegressorBlock(
        i, J_body, J_body_dot, R_body,
        a_local, omega_local, alpha_local);

    std::size_t offset = i * InertialParams::PARAMS_PER_BODY;
    Y.block(0, offset, n_dof_, InertialParams::PARAMS_PER_BODY) = Y_body;
  }

  // ---- Damping 列 ----
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
