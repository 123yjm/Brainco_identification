#include "revoarm_new_regressor.hpp"
#include <iostream>

namespace robot_dynamics {

// 前方 kinematic-only body 的数量 (base_link + right_base_link)
static constexpr std::size_t KINEMATIC_PREFIX = 2;

// ============================================================================
// 构造函数
// ============================================================================

RevoarmNewRegressor::RevoarmNewRegressor() { initBodies(); }

void RevoarmNewRegressor::initBodies() {
  // --- kinematic prefix: bodies_[0] = base_link ---
  bodies_[0].name = "base_link";
  bodies_[0].mass = 0;
  bodies_[0].has_joint = false;

  // --- kinematic prefix: bodies_[1] = right_base_link ---
  bodies_[1].name = "right_base_link";
  bodies_[1].pos = Vector3d(0.0055, -0.10424, -0.038037);
  bodies_[1].quat = Quaterniond(0.996194825565, -0.0871542857135, 0.0, 0.0);
  bodies_[1].mass = 0;
  bodies_[1].has_joint = false;

  // ========== 7 regressor bodies (indices 2..8) ==========

  // --- regressor body 0: right_arm_link1 ---
  bodies_[2].name = "right_arm_link1";
  bodies_[2].mass = 0.99862;
  bodies_[2].com = Vector3d(0.00138261246537, -0.0527123429468, -0.00121652148011);
  bodies_[2].Ixx = 0.00134544913222;
  bodies_[2].Iyy = 0.000834135586928;
  bodies_[2].Izz = 0.00103343870055;
  bodies_[2].Ixy = 9.35736730946e-06;
  bodies_[2].Ixz = 3.47820470915e-07;
  bodies_[2].Iyz = 4.90367363186e-05;
  bodies_[2].joint_axis = Vector3d(0.0, 1.0, 0.0);
  bodies_[2].damping = 0.179902637204222;
  bodies_[2].has_joint = true;

  // --- regressor body 1: right_arm_link2 ---
  bodies_[3].name = "right_arm_link2";
  bodies_[3].pos = Vector3d(0.0, -0.06, 0.0);
  bodies_[3].quat = Quaterniond(0.996194825565, 0.0871542857135, 0.0, 0.0);
  bodies_[3].mass = 0.1942;
  bodies_[3].com = Vector3d(5.45283276618e-06, -0.000282431180944, -0.0268913421348);
  bodies_[3].Ixx = 0.000382625514425;
  bodies_[3].Iyy = 0.000369233743661;
  bodies_[3].Izz = 0.000207280501913;
  bodies_[3].Ixy = 9.48679552051e-07;
  bodies_[3].Ixz = -2.1848535528e-06;
  bodies_[3].Iyz = 2.31382379314e-06;
  bodies_[3].joint_axis = Vector3d(-1.0, 0.0, 0.0);
  bodies_[3].damping = 0.230292969031887;
  bodies_[3].has_joint = true;

  // --- regressor body 2: right_arm_link3 ---
  bodies_[4].name = "right_arm_link3";
  bodies_[4].pos = Vector3d(0.0, 0.0, -0.099);
  bodies_[4].mass = 0.56897;
  bodies_[4].com = Vector3d(-0.000454569614792, 3.12449787288e-05, -0.0666939671661);
  bodies_[4].Ixx = 0.00134243655403;
  bodies_[4].Iyy = 0.00131310898292;
  bodies_[4].Izz = 0.000492154320293;
  bodies_[4].Ixy = -1.10159695691e-06;
  bodies_[4].Ixz = -8.9867271334e-06;
  bodies_[4].Iyz = 1.87663764445e-06;
  bodies_[4].joint_axis = Vector3d(0.0, 0.0, 1.0);
  bodies_[4].damping = 0.216132996448927;
  bodies_[4].has_joint = true;

  // --- regressor body 3: right_arm_link4 ---
  bodies_[5].name = "right_arm_link4";
  bodies_[5].pos = Vector3d(0.0, 0.0, -0.121);
  bodies_[5].mass = 0.43039;
  bodies_[5].com = Vector3d(-7.58557077911e-06, 0.000351070451149, -0.0253844870788);
  bodies_[5].Ixx = 0.00047306084261;
  bodies_[5].Iyy = 0.00047377939084;
  bodies_[5].Izz = 0.000275999880013;
  bodies_[5].Ixy = 1.33236901297e-07;
  bodies_[5].Ixz = -1.09059920161e-06;
  bodies_[5].Iyz = -2.6711329621e-06;
  bodies_[5].joint_axis = Vector3d(0.0, -1.0, 0.0);
  bodies_[5].damping = 0.283625824431603;
  bodies_[5].has_joint = true;

  // --- regressor body 4: right_arm_link5 ---
  bodies_[6].name = "right_arm_link5";
  bodies_[6].pos = Vector3d(0.0, 0.0, -0.0635);
  bodies_[6].mass = 0.80969;
  bodies_[6].com = Vector3d(0.000915076828994, 1.83496866724e-05, -0.0751470282845);
  bodies_[6].Ixx = 0.00182931151009;
  bodies_[6].Iyy = 0.00183450989562;
  bodies_[6].Izz = 0.00056086991064;
  bodies_[6].Ixy = 9.37494132217e-07;
  bodies_[6].Ixz = 3.67371332139e-05;
  bodies_[6].Iyz = -1.6270176108e-06;
  bodies_[6].joint_axis = Vector3d(0.0, 0.0, 1.0);
  bodies_[6].damping = 0.0862977234677901;
  bodies_[6].has_joint = true;

  // --- regressor body 5: right_arm_link6 ---
  bodies_[7].name = "right_arm_link6";
  bodies_[7].pos = Vector3d(0.0, 0.0, -0.1525);
  bodies_[7].mass = 0.31466;
  bodies_[7].com = Vector3d(0.000179459444033, 0.00100056983205, -2.47524159457e-05);
  bodies_[7].Ixx = 0.000120895453787;
  bodies_[7].Iyy = 0.000136714381567;
  bodies_[7].Izz = 0.000137495744014;
  bodies_[7].Ixy = 2.38559203994e-07;
  bodies_[7].Ixz = -7.17472862339e-08;
  bodies_[7].Iyz = 2.58366532259e-07;
  bodies_[7].joint_axis = Vector3d(1.0, 0.0, 0.0);
  bodies_[7].damping = 0.211563663937835;
  bodies_[7].has_joint = true;

  // --- regressor body 6: right_arm_link7 ---
  // (合并了 connector_link + hand_base_link 的质量和惯量)
  bodies_[8].name = "right_arm_link7";
  bodies_[8].quat = Quaterniond(0.999999529356, 0.000970199847794, 0.0, 0.0);
  bodies_[8].mass = 0.73111;
  bodies_[8].com = Vector3d(-0.000732931223882178, -0.00532077017212816, -0.0449146805909785);
  bodies_[8].Ixx = 0.00125625176555862;
  bodies_[8].Iyy = 0.00116418038751659;
  bodies_[8].Izz = 0.000252744963516969;
  bodies_[8].Ixy = 2.18319789240444e-05;
  bodies_[8].Ixz = 8.15936869882136e-05;
  bodies_[8].Iyz = 5.51214029071445e-05;
  bodies_[8].joint_axis = Vector3d(0.0, -1.0, 0.0);
  bodies_[8].damping = 0.300442759539957;
  bodies_[8].has_joint = true;

  // armature = 0 (MJCF 未指定)
  for (std::size_t i = 0; i < KINEMATIC_PREFIX + N_BODIES; ++i) {
    bodies_[i].armature = 0.0;
  }
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
  const std::size_t total = KINEMATIC_PREFIX + N_BODIES;
  std::vector<Matrix4d> transforms(total);

  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  std::size_t joint_idx = 0;

  for (std::size_t i = 1; i < total; ++i) {
    const auto &body = bodies_[i];
    Matrix4d T_parent_body = poseToTransform(body.pos, body.quat);

    if (body.has_joint && joint_idx < N_DOF) {
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
  MatrixXd J = MatrixXd::Zero(6, N_DOF);
  auto transforms = computeBodyTransforms(q);
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3);

  const std::size_t total = KINEMATIC_PREFIX + N_BODIES;
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
  MatrixXd J = MatrixXd::Zero(6, N_DOF);
  auto transforms = computeBodyTransforms(q);
  Vector3d p_body = computeBodyCOM(body_idx, q);

  const std::size_t total = KINEMATIC_PREFIX + N_BODIES;
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
  std::size_t params = N_BODIES * InertialParams::PARAMS_PER_BODY;

  if (hasFlag(flags, ParamFlags::ARMATURE)) {
    params += N_DOF;
  }

  if (hasFlag(flags, ParamFlags::DAMPING)) {
    params += N_DOF;
  }

  return params;
}

RevoarmNewRegressor::VectorXd
RevoarmNewRegressor::computeParameterVector(ParamFlags flags) const {
  const std::size_t num_params = numParameters(flags);
  VectorXd theta = VectorXd::Zero(num_params);

  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + KINEMATIC_PREFIX;
    auto sip = InertialParams::fromBody(bodies_[body_idx]);
    auto sip_vec = sip.toVector();
    std::size_t offset = i * InertialParams::PARAMS_PER_BODY;
    theta.segment(offset, InertialParams::PARAMS_PER_BODY) = sip_vec;
  }

  std::size_t current_offset = N_BODIES * InertialParams::PARAMS_PER_BODY;

  if (hasFlag(flags, ParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = bodies_[i + KINEMATIC_PREFIX].armature;
    }
    current_offset += N_DOF;
  }

  if (hasFlag(flags, ParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = bodies_[i + KINEMATIC_PREFIX].damping;
    }
  }

  return theta;
}

std::vector<std::string>
RevoarmNewRegressor::getParameterNames(ParamFlags flags) const {
  std::vector<std::string> names;

  const char *body_names[] = {
      "link1", "link2", "link3", "link4", "link5", "link6", "link7"};

  const char *param_names[] = {"m",   "mx",  "my",  "mz",  "Ixx",
                               "Ixy", "Ixz", "Iyy", "Iyz", "Izz"};

  for (std::size_t i = 0; i < N_BODIES; ++i) {
    for (int j = 0; j < 10; ++j) {
      names.push_back(std::string(body_names[i]) + "_" + param_names[j]);
    }
  }

  if (hasFlag(flags, ParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      names.push_back("armature_" + std::to_string(i + 1));
    }
  }

  if (hasFlag(flags, ParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
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
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd,
    const VectorXd &qdd) const {

  auto transforms = computeBodyTransforms(q);
  Matrix3d R = transforms[body_idx].block<3, 3>(0, 0);
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3);

  // Body 原点的雅可比 (世界坐标系)
  MatrixXd J_world = MatrixXd::Zero(6, N_DOF);
  const std::size_t total = KINEMATIC_PREFIX + N_BODIES;
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

  // 雅可比导数（数值微分）
  const double dt = 1e-7;
  VectorXd q_plus = q + dt * qd;
  auto transforms_plus = computeBodyTransforms(q_plus);
  Vector3d p_origin_plus = transforms_plus[body_idx].block<3, 1>(0, 3);

  MatrixXd J_world_plus = MatrixXd::Zero(6, N_DOF);
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

  // 构建回归矩阵块 Y_body (N_DOF × 10)
  MatrixXd Y_block = MatrixXd::Zero(N_DOF, 10);

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
  MatrixXd Y = MatrixXd::Zero(N_DOF, num_params);

  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + KINEMATIC_PREFIX;
    MatrixXd Y_body = computeBodyRegressorBlock(body_idx, q, qd, qdd);
    std::size_t offset = i * InertialParams::PARAMS_PER_BODY;
    Y.block(0, offset, N_DOF, InertialParams::PARAMS_PER_BODY) = Y_body;
  }

  std::size_t current_offset = N_BODIES * InertialParams::PARAMS_PER_BODY;

  if (hasFlag(flags, ParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      Y(i, current_offset + i) = qdd(i);
    }
    current_offset += N_DOF;
  }

  if (hasFlag(flags, ParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
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
  MatrixXd W = MatrixXd::Zero(N_DOF * K, num_params);

#ifdef IDENTIFICATION_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(K); ++k) {
    VectorXd q = Q.col(k);
    VectorXd qd = Qd.col(k);
    VectorXd qdd = Qdd.col(k);
    MatrixXd Y_k = computeRegressorMatrix(q, qd, qdd, flags);
    W.block(k * static_cast<Eigen::Index>(N_DOF), 0,
            static_cast<Eigen::Index>(N_DOF),
            static_cast<Eigen::Index>(num_params)) = Y_k;
  }

  return W;
}

} // namespace robot_dynamics
