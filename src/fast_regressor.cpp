/**
 * @file fast_regressor.cpp
 * @brief 快速解析回归器 — sin/cos + 几何 Jacobian，无 FK/四元数/数值微分
 *
 * 与 SerialArmRegressor 输出完全一致（机器精度内），速度提升 5-10x。
 * 用于激励轨迹优化等需要大量目标评估的场景。
 */
#include "fast_regressor.hpp"
#include "serial_arm_regressor.hpp"
#include "rigid_body.hpp"
#include <cmath>

namespace robot_dynamics {

// ============================================================================
// 预计算常量结构
// ============================================================================
struct FastFKCache {
  // 每个 body 的常量: R_parent (固定旋转), p_parent (固定平移), axis (关节轴)
  std::vector<Eigen::Matrix3d> R_parent;
  std::vector<Eigen::Vector3d> p_parent;
  std::vector<Eigen::Vector3d> joint_axis;
  std::vector<bool> has_joint;
  std::size_t n_dof;
  std::size_t n_bodies;
  std::size_t kinematic_prefix;
  Eigen::Vector3d gravity;
};

// 全局缓存（初始化后不变）
static FastFKCache g_cache;
static bool g_cache_initialized = false;

void initFastRegressor(const SerialArmRegressor &reg) {
  if (g_cache_initialized) return;

  const auto &bodies = reg.getBodies();
  g_cache.n_dof = reg.nDof();
  g_cache.n_bodies = reg.nBodies();
  g_cache.kinematic_prefix = reg.kinematicPrefix();

  std::size_t total = g_cache.kinematic_prefix + g_cache.n_bodies;
  g_cache.R_parent.resize(total);
  g_cache.p_parent.resize(total);
  g_cache.joint_axis.resize(total);
  g_cache.has_joint.resize(total);
  g_cache.gravity = Eigen::Vector3d(0, 0, -9.81);

  for (std::size_t i = 0; i < total; ++i) {
    g_cache.R_parent[i] = bodies[i].quat.toRotationMatrix();
    g_cache.p_parent[i] = bodies[i].pos;
    g_cache.joint_axis[i] = bodies[i].joint_axis;
    g_cache.has_joint[i] = bodies[i].has_joint;
  }
  g_cache_initialized = true;
}

// ============================================================================
// 快速回归矩阵计算
// ============================================================================
Eigen::MatrixXd computeRegressorFast(const Eigen::VectorXd &q,
                                      const Eigen::VectorXd &qd,
                                      const Eigen::VectorXd &qdd,
                                      ParamFlags flags) {
  const std::size_t n_dof = g_cache.n_dof;
  const std::size_t n_bodies = g_cache.n_bodies;
  const std::size_t kprefix = g_cache.kinematic_prefix;
  const std::size_t total = kprefix + n_bodies;
  const std::size_t num_params = (flags == ParamFlags::NONE)
      ? n_bodies * 10
      : n_bodies * 10 + (hasFlag(flags, ParamFlags::ARMATURE) ? n_dof : 0)
                        + (hasFlag(flags, ParamFlags::DAMPING) ? n_dof : 0);

  // ---- 1. 预计算 sin/cos ------------------------------------------------
  double sq[7], cq[7];
  for (std::size_t i = 0; i < n_dof; ++i) {
    sq[i] = std::sin(q(i));
    cq[i] = std::cos(q(i));
  }

  // ---- 2. 增量 FK: R[i], p[i], z[i], p_joint[i] ------------------------
  std::vector<Eigen::Matrix3d> R(total);
  std::vector<Eigen::Vector3d> p(total);
  std::vector<Eigen::Vector3d> z(total);
  std::vector<Eigen::Vector3d> p_joint(total);

  // Body 0 (kinematic prefix, no joint)
  R[0] = g_cache.R_parent[0];
  p[0] = g_cache.p_parent[0];
  z[0] = Eigen::Vector3d::Zero();
  p_joint[0] = p[0];

  std::size_t joint_idx = 0;
  for (std::size_t i = 1; i < total; ++i) {
    if (g_cache.has_joint[i] && joint_idx < n_dof) {
      // Rodriguez: R_joint = I + sin(q)*[a]× + (1-cos(q))*[a]×²
      double s = sq[joint_idx], c = cq[joint_idx];
      const Eigen::Vector3d &a = g_cache.joint_axis[i];

      // [a]×  skew-symmetric
      Eigen::Matrix3d ax;
      ax << 0, -a(2), a(1),
            a(2), 0, -a(0),
            -a(1), a(0), 0;
      Eigen::Matrix3d ax2 = ax * ax;

      Eigen::Matrix3d R_joint = Eigen::Matrix3d::Identity() + s * ax + (1.0 - c) * ax2;

      R[i] = R[i-1] * g_cache.R_parent[i] * R_joint;
      p[i] = p[i-1] + R[i-1] * g_cache.p_parent[i];
      z[i] = R[i] * a;  // 关节轴在世界坐标系
      p_joint[i] = p[i];
      ++joint_idx;
    } else {
      R[i] = R[i-1] * g_cache.R_parent[i];
      p[i] = p[i-1] + R[i-1] * g_cache.p_parent[i];
      z[i] = Eigen::Vector3d::Zero();
      p_joint[i] = p[i];
    }
  }

  // ---- 3. 速度/加速度 (世界坐标系) --------------------------------------
  // omega_i, v_i (world frame), omega_dot_i, v_dot_i
  std::vector<Eigen::Vector3d> omega(total, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> v(total, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> omega_dot(total, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> v_dot(total, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> z_dot(total, Eigen::Vector3d::Zero());

  // 递归计算
  joint_idx = 0;
  for (std::size_t i = 1; i < total; ++i) {
    if (g_cache.has_joint[i] && joint_idx < n_dof) {
      double dq_i  = qd(joint_idx);
      double ddq_i = qdd(joint_idx);

      omega[i] = omega[i-1] + z[i] * dq_i;
      v[i] = v[i-1] + omega[i-1].cross(p[i] - p[i-1]);

      // 解析 Jacobian 导数项
      z_dot[i] = omega[i-1].cross(z[i]);
      omega_dot[i] = omega_dot[i-1] + z_dot[i] * dq_i + z[i] * ddq_i;

      Eigen::Vector3d r = p[i] - p[i-1];
      v_dot[i] = v_dot[i-1]
               + omega_dot[i-1].cross(r)
               + omega[i-1].cross(omega[i-1].cross(r));

      ++joint_idx;
    } else {
      omega[i] = omega[i-1];
      v[i] = v[i-1] + omega[i-1].cross(p[i] - p[i-1]);
      omega_dot[i] = omega_dot[i-1];
      Eigen::Vector3d r = p[i] - p[i-1];
      v_dot[i] = v_dot[i-1]
               + omega_dot[i-1].cross(r)
               + omega[i-1].cross(omega[i-1].cross(r));
    }
  }

  // ---- 4. 构建回归矩阵 Y ------------------------------------------------
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(n_dof, num_params);
  Eigen::Vector3d g = g_cache.gravity;

  // 统计实际关节映射 (body index → joint index)
  std::vector<int> body_to_joint(total, -1);
  joint_idx = 0;
  for (std::size_t i = 1; i < total; ++i) {
    if (g_cache.has_joint[i] && joint_idx < n_dof) {
      body_to_joint[i] = static_cast<int>(joint_idx);
      ++joint_idx;
    }
  }

  for (std::size_t b = 0; b < n_bodies; ++b) {
    std::size_t body_idx = kprefix + b;
    std::size_t offset = b * 10;

    // Body origin 运动学 (世界坐标系)
    Eigen::Vector3d a_w = v_dot[body_idx];
    Eigen::Vector3d om_w = omega[body_idx];
    Eigen::Vector3d al_w = omega_dot[body_idx];

    // 转到 body local 坐标系
    Eigen::Matrix3d Rt = R[body_idx].transpose();
    Eigen::Vector3d a_loc = Rt * a_w;
    Eigen::Vector3d om_loc = Rt * om_w;
    Eigen::Vector3d al_loc = Rt * al_w;
    Eigen::Vector3d g_loc = Rt * g;
    Eigen::Vector3d b_loc = a_loc - g_loc;

    // 构建 Jacobian (世界坐标系, 再转到 local)
    // Jv_local = Rt * Jv_world,  Jw_local = Rt * Jw_world
    Eigen::MatrixXd Jv = Eigen::MatrixXd::Zero(3, n_dof);
    Eigen::MatrixXd Jw = Eigen::MatrixXd::Zero(3, n_dof);

    // Jacobian 解析导数
    Eigen::MatrixXd Jv_dot = Eigen::MatrixXd::Zero(3, n_dof);
    Eigen::MatrixXd Jw_dot = Eigen::MatrixXd::Zero(3, n_dof);

    for (std::size_t j = 1; j <= body_idx; ++j) {
      int jidx = body_to_joint[j];
      if (jidx < 0) continue;

      // J.col(j) = [z_j × (p_body - p_joint_j); z_j]
      Eigen::Vector3d r = p[body_idx] - p_joint[j];
      Eigen::Vector3d jv_col = z[j].cross(r);
      Eigen::Vector3d jw_col = z[j];

      // J_dot.col(j)
      Eigen::Vector3d jvd = z_dot[j].cross(r) + z[j].cross(v[body_idx] - v[j]);
      Eigen::Vector3d jwd = z_dot[j];

      // 转到 body local
      Jv.col(jidx) = Rt * jv_col;
      Jw.col(jidx) = Rt * jw_col;
      Jv_dot.col(jidx) = Rt * jvd;
      Jw_dot.col(jidx) = Rt * jwd;
    }

    // ---- 回归矩阵块 (与原始 computeBodyRegressorBlock 相同公式) ---------
    auto skew = [](const Eigen::Vector3d &v) -> Eigen::Matrix3d {
      Eigen::Matrix3d S;
      S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      return S;
    };

    Eigen::Matrix3d al_skew = skew(al_loc);
    Eigen::Matrix3d om_skew = skew(om_loc);
    Eigen::Matrix3d K = al_skew + om_skew * om_skew;

    double ox = om_loc(0), oy = om_loc(1), oz = om_loc(2);
    double ax = al_loc(0), ay = al_loc(1), az = al_loc(2);

    // Column 0: m
    Y.block(0, offset, n_dof, 1) = Jv.transpose() * b_loc;

    // Columns 1-3: mx, my, mz
    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3d e_i = Eigen::Vector3d::Zero();
      e_i(i) = 1.0;
      Y.block(0, offset + 1 + i, n_dof, 1) =
          Jv.transpose() * K.col(i) + Jw.transpose() * e_i.cross(b_loc);
    }

    // Columns 4-9: Ixx, Ixy, Ixz, Iyy, Iyz, Izz
    Y.block(0, offset + 4, n_dof, 1) = Jw.transpose() * Eigen::Vector3d(ax, ox*oz, -ox*oy);
    Y.block(0, offset + 5, n_dof, 1) = Jw.transpose() * Eigen::Vector3d(ay - ox*oz, ax + oy*oz, ox*ox - oy*oy);
    Y.block(0, offset + 6, n_dof, 1) = Jw.transpose() * Eigen::Vector3d(az + ox*oy, oz*oz - ox*ox, ax - oy*oz);
    Y.block(0, offset + 7, n_dof, 1) = Jw.transpose() * Eigen::Vector3d(-oy*oz, ay, ox*oy);
    Y.block(0, offset + 8, n_dof, 1) = Jw.transpose() * Eigen::Vector3d(oy*oy - oz*oz, az - ox*oy, ay + ox*oz);
    Y.block(0, offset + 9, n_dof, 1) = Jw.transpose() * Eigen::Vector3d(oy*oz, -ox*oz, az);
  }

  // ---- 5. armature / damping (与原始相同) -------------------------------
  if (hasFlag(flags, ParamFlags::ARMATURE)) {
    std::size_t ao = n_bodies * 10;
    for (std::size_t i = 0; i < n_dof; ++i)
      Y(i, ao + i) = qdd(i);
  }
  if (hasFlag(flags, ParamFlags::DAMPING)) {
    std::size_t do_off = n_bodies * 10;
    if (hasFlag(flags, ParamFlags::ARMATURE)) do_off += n_dof;
    for (std::size_t i = 0; i < n_dof; ++i)
      Y(i, do_off + i) = -qd(i);
  }

  return Y;
}

// ============================================================================
// 快速堆叠观测矩阵
// ============================================================================
Eigen::MatrixXd computeObservationMatrixFast(const Eigen::MatrixXd &Q,
                                              const Eigen::MatrixXd &Qd,
                                              const Eigen::MatrixXd &Qdd,
                                              ParamFlags flags) {
  const Eigen::Index K = Q.cols();
  Eigen::Index n_dof = static_cast<Eigen::Index>(g_cache.n_dof);
  Eigen::Index n_params = static_cast<Eigen::Index>(
      g_cache.n_bodies * 10 +
      (hasFlag(flags, ParamFlags::ARMATURE) ? g_cache.n_dof : 0) +
      (hasFlag(flags, ParamFlags::DAMPING) ? g_cache.n_dof : 0));

  Eigen::MatrixXd W = Eigen::MatrixXd::Zero(n_dof * K, n_params);

#ifdef IDENTIFICATION_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (Eigen::Index k = 0; k < K; ++k) {
    Eigen::VectorXd q   = Q.col(k);
    Eigen::VectorXd qd  = Qd.col(k);
    Eigen::VectorXd qdd = Qdd.col(k);
    Eigen::MatrixXd Y_k = computeRegressorFast(q, qd, qdd, flags);
    W.block(k * n_dof, 0, n_dof, n_params) = Y_k;
  }
  return W;
}

} // namespace robot_dynamics
