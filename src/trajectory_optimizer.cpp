/**
 * @file trajectory_optimizer.cpp
 * @brief 傅里叶激励轨迹优化 — 实现
 */

#include "trajectory_optimizer.hpp"
#include "regressor_factory.hpp"

#include <nlopt.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace excitation_trajectory {

// ============================================================================
// 辅助: YAML 序列 → Eigen::VectorXd
// ============================================================================

static Eigen::VectorXd parseVectorXd(const YAML::Node &node) {
  if (!node || !node.IsSequence()) {
    throw std::runtime_error("parseVectorXd: 需要 YAML 序列");
  }
  Eigen::VectorXd v(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    v(static_cast<Eigen::Index>(i)) = node[i].as<double>();
  }
  return v;
}

// ============================================================================
// FourierJointParams 实现
// ============================================================================

void FourierJointParams::evaluate(double t, double wf, double &q_out,
                                   double &qd_out, double &qdd_out) const {
  // 预计算各阶 sin/cos
  double sin_vals[kOrder];
  double cos_vals[kOrder];
  for (int l = 0; l < kOrder; ++l) {
    double theta = static_cast<double>(l + 1) * wf * t;
    sin_vals[l] = std::sin(theta);
    cos_vals[l] = std::cos(theta);
  }

  double q = c;
  double qd = 0.0;
  double qdd = 0.0;

  for (int l = 0; l < kOrder; ++l) {
    double l_wf = static_cast<double>(l + 1) * wf;
    double inv_l_wf = 1.0 / l_wf;

    q += a(l) * sin_vals[l] * inv_l_wf - b(l) * cos_vals[l] * inv_l_wf;
    qd += a(l) * cos_vals[l] + b(l) * sin_vals[l];
    qdd += -a(l) * l_wf * sin_vals[l] + b(l) * l_wf * cos_vals[l];
  }

  q_out = q;
  qd_out = qd;
  qdd_out = qdd;
}

double FourierJointParams::velocityAmplitudeBound() const {
  double bound = 0.0;
  for (int l = 0; l < kOrder; ++l) {
    bound += std::sqrt(a(l) * a(l) + b(l) * b(l));
  }
  return bound;
}

double FourierJointParams::accelerationAmplitudeBound(double wf) const {
  double bound = 0.0;
  for (int l = 0; l < kOrder; ++l) {
    bound += std::sqrt(a(l) * a(l) + b(l) * b(l)) *
             static_cast<double>(l + 1) * wf;
  }
  return bound;
}

double FourierJointParams::positionAmplitudeBound(double wf) const {
  double bound = 0.0;
  for (int l = 0; l < kOrder; ++l) {
    bound += std::sqrt(a(l) * a(l) + b(l) * b(l)) /
             (static_cast<double>(l + 1) * wf);
  }
  return bound;
}

double FourierJointParams::qAtZero(double wf) const {
  double q0 = c;
  for (int l = 0; l < kOrder; ++l) {
    q0 -= b(l) / (static_cast<double>(l + 1) * wf);
  }
  return q0;
}

double FourierJointParams::qdAtZero() const {
  double qd0 = 0.0;
  for (int l = 0; l < kOrder; ++l) {
    qd0 += a(l);
  }
  return qd0;
}

double FourierJointParams::qddAtZero(double wf) const {
  double qdd0 = 0.0;
  for (int l = 0; l < kOrder; ++l) {
    qdd0 += b(l) * static_cast<double>(l + 1) * wf;
  }
  return qdd0;
}

// ============================================================================
// FourierTrajectoryParams 实现
// ============================================================================

FourierTrajectoryParams
FourierTrajectoryParams::fromVector(const Eigen::VectorXd &x, int dof,
                                     int order, double wf) {
  FourierTrajectoryParams params;
  params.dof = dof;
  params.order = order;
  params.wf = wf;
  params.joints.resize(dof);

  const int N = order;
  const int D = dof;

  for (int j = 0; j < D; ++j) {
    auto &joint = params.joints[j];
    for (int l = 0; l < N; ++l) {
      joint.a(l) = x(l * D + j);
      joint.b(l) = x(N * D + l * D + j);
    }
    joint.c = x(2 * N * D + j);
  }

  return params;
}

Eigen::VectorXd FourierTrajectoryParams::toVector() const {
  const int D = dof;
  const int N = order;
  Eigen::VectorXd x(2 * N * D + D);

  for (int l = 0; l < N; ++l) {
    for (int j = 0; j < D; ++j) {
      x(l * D + j) = joints[j].a(l);
      x(N * D + l * D + j) = joints[j].b(l);
    }
  }
  for (int j = 0; j < D; ++j) {
    x(2 * N * D + j) = joints[j].c;
  }

  return x;
}

void FourierTrajectoryParams::evaluateAll(
    double t, Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> qd,
    Eigen::Ref<Eigen::VectorXd> qdd) const {
  for (int j = 0; j < dof; ++j) {
    joints[j].evaluate(t, wf, q(j), qd(j), qdd(j));
  }
}

Eigen::VectorXd FourierTrajectoryParams::velocityAmplitudeBound() const {
  Eigen::VectorXd bound(dof);
  for (int j = 0; j < dof; ++j) {
    bound(j) = joints[j].velocityAmplitudeBound();
  }
  return bound;
}

Eigen::VectorXd FourierTrajectoryParams::accelerationAmplitudeBound() const {
  Eigen::VectorXd bound(dof);
  for (int j = 0; j < dof; ++j) {
    bound(j) = joints[j].accelerationAmplitudeBound(wf);
  }
  return bound;
}

Eigen::VectorXd FourierTrajectoryParams::positionAmplitudeBound() const {
  Eigen::VectorXd bound(dof);
  for (int j = 0; j < dof; ++j) {
    bound(j) = joints[j].positionAmplitudeBound(wf);
  }
  return bound;
}

void FourierTrajectoryParams::evaluateAtZero(
    Eigen::Ref<Eigen::VectorXd> q0, Eigen::Ref<Eigen::VectorXd> qd0,
    Eigen::Ref<Eigen::VectorXd> qdd0) const {
  for (int j = 0; j < dof; ++j) {
    q0(j) = joints[j].qAtZero(wf);
    qd0(j) = joints[j].qdAtZero();
    qdd0(j) = joints[j].qddAtZero(wf);
  }
}

// ============================================================================
// ExcitationTrajectoryOptimizer 实现
// ============================================================================

ExcitationTrajectoryOptimizer::ExcitationTrajectoryOptimizer(
    const ExcitationTrajectoryConfig &cfg)
    : cfg_(cfg) {
  dof_ = static_cast<int>(cfg_.q_init.size());
  n_vars_ = dof_ * (2 * cfg_.fourier_order + 1);

  // 通过工厂创建回归器
  regressor_ = robot_dynamics::RegressorFactory::create(cfg_.robot_name,
                                                         cfg_.kinematic_params_path);
  if (!regressor_) {
    throw std::runtime_error("无法创建机器人回归器: " + cfg_.robot_name);
  }

  auto flags = robot_dynamics::ParamFlags::ALL;
  num_params_ = static_cast<int>(regressor_->numParameters(flags));

  std::cout << "激励轨迹优化初始化:" << std::endl;
  std::cout << "  机器人: " << cfg_.robot_name << std::endl;
  std::cout << "  DOF: " << dof_ << std::endl;
  std::cout << "  参数数: " << num_params_ << std::endl;
  std::cout << "  优化变量数: " << n_vars_ << std::endl;
  std::cout << "  傅里叶阶数: " << cfg_.fourier_order << std::endl;
  std::cout << "  运行时长: " << cfg_.sampling_time
            << " s,  wf: " << cfg_.wf << " rad/s" << std::endl;
  std::cout << "  优化采样: " << cfg_.sampling_frequency << " Hz  → "
            << cfg_.num_timesteps
            << " 点 (dt=" << 1.0 / cfg_.sampling_frequency << " s)" << std::endl;
  std::cout << "  输出频率: " << cfg_.trajectory_frequency << " Hz  → "
            << static_cast<int>(cfg_.sampling_time * cfg_.trajectory_frequency) + 1
            << " 点 (dt=" << 1.0 / cfg_.trajectory_frequency << " s)" << std::endl;
  std::cout << "  正则化 lambda: " << cfg_.regularization_lambda << std::endl;
  std::cout << "  多起点次数: " << cfg_.multi_start_count << std::endl;
}

// ---------------------------------------------------------------------------
// 目标函数计算
// ---------------------------------------------------------------------------

double
ExcitationTrajectoryOptimizer::computeObjective(const Eigen::VectorXd &x) const {
  // 1. 解码傅里叶参数
  auto params = FourierTrajectoryParams::fromVector(x, dof_, cfg_.fourier_order,
                                                     cfg_.wf);

  // 2. 构建 Q, Qd, Qdd 矩阵 (dof × K, 每列一个采样点)
  const int K = cfg_.num_timesteps;
  const double dt = cfg_.sampling_time / static_cast<double>(K - 1);

  Eigen::MatrixXd Q(dof_, K);
  Eigen::MatrixXd Qd(dof_, K);
  Eigen::MatrixXd Qdd(dof_, K);

  Eigen::VectorXd q_tmp(dof_), qd_tmp(dof_), qdd_tmp(dof_);
  for (int k = 0; k < K; ++k) {
    double t = static_cast<double>(k) * dt;
    params.evaluateAll(t, q_tmp, qd_tmp, qdd_tmp);
    Q.col(k) = q_tmp;
    Qd.col(k) = qd_tmp;
    Qdd.col(k) = qdd_tmp;
  }

  // 3. 通过 regressor 构建堆叠观测矩阵 W
  auto flags = robot_dynamics::ParamFlags::ALL;
  Eigen::MatrixXd W =
      regressor_->computeObservationMatrix(Q, Qd, Qdd, flags);

  // 4. D-最优准则: max det(W^T W)  ⇔  min -log(det(W^T W))
  //    使用特征值对数之和避免数值溢出
  Eigen::MatrixXd WtW = W.transpose() * W;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigs(WtW);
  if (eigs.info() != Eigen::Success) {
    return 1e10; // 数值问题时返回大值
  }

  const Eigen::VectorXd &evals = eigs.eigenvalues();

  // 极小特征值截断，避免 log(0) 或 log(负)
  double sum_log = 0.0;
  for (Eigen::Index i = 0; i < evals.size(); ++i) {
    double ev = std::max(evals(i), 1e-12);
    sum_log += std::log(ev);
  }

  return -sum_log; // 最小化负的对数行列式
}

// ---------------------------------------------------------------------------
// NLopt 回调
// ---------------------------------------------------------------------------

double ExcitationTrajectoryOptimizer::objective(unsigned n, const double *x,
                                                 double * /*grad*/, void *data) {
  auto *self = static_cast<ExcitationTrajectoryOptimizer *>(data);
  Eigen::VectorXd x_vec =
      Eigen::VectorXd::Map(x, static_cast<Eigen::Index>(n));
  return self->computeObjective(x_vec);
}

void ExcitationTrajectoryOptimizer::eqConstraints(
    unsigned m, double *result, unsigned n, const double *x, double * /*grad*/,
    void *data) {
  auto *self = static_cast<ExcitationTrajectoryOptimizer *>(data);
  Eigen::VectorXd x_vec =
      Eigen::VectorXd::Map(x, static_cast<Eigen::Index>(n));

  auto params = FourierTrajectoryParams::fromVector(x_vec, self->dof_,
                                                     self->cfg_.fourier_order,
                                                     self->cfg_.wf);

  Eigen::VectorXd q0(self->dof_), qd0(self->dof_), qdd0(self->dof_);
  params.evaluateAtZero(q0, qd0, qdd0);

  // 约束布局: [q(0)-q_init, qd(0), qdd(0)]  = 0
  // 共 3*dof_ 个等式约束
  unsigned idx = 0;
  for (int j = 0; j < self->dof_; ++j) {
    result[idx++] = q0(j) - self->cfg_.q_init(j);
  }
  for (int j = 0; j < self->dof_; ++j) {
    result[idx++] = qd0(j); // qd(0) = 0
  }
  for (int j = 0; j < self->dof_; ++j) {
    result[idx++] = qdd0(j); // qdd(0) = 0
  }
}

void ExcitationTrajectoryOptimizer::ineqConstraints(
    unsigned m, double *result, unsigned n, const double *x, double * /*grad*/,
    void *data) {
  auto *self = static_cast<ExcitationTrajectoryOptimizer *>(data);
  Eigen::VectorXd x_vec =
      Eigen::VectorXd::Map(x, static_cast<Eigen::Index>(n));

  auto params = FourierTrajectoryParams::fromVector(x_vec, self->dof_,
                                                     self->cfg_.fourier_order,
                                                     self->cfg_.wf);

  // 幅值上界
  Eigen::VectorXd vel_bound = params.velocityAmplitudeBound();
  Eigen::VectorXd acc_bound = params.accelerationAmplitudeBound();
  Eigen::VectorXd pos_bound = params.positionAmplitudeBound();

  // 约束布局: [pos_lower, pos_upper, vel_ineq, acc_ineq] <= 0
  // 位置约束拆成两个不等式以避免 |·| 导致的不可微性：
  //   c_j - q_max_j + pos_amp_j <= 0   (c_j + pos_amp_j <= q_max_j)
  //   q_min_j - c_j + pos_amp_j <= 0   (-c_j + pos_amp_j <= -q_min_j)
  unsigned idx = 0;
  for (int j = 0; j < self->dof_; ++j) {
    result[idx++] = params.joints[j].c + pos_bound(j) - self->cfg_.q_max(j);
  }
  for (int j = 0; j < self->dof_; ++j) {
    result[idx++] = -params.joints[j].c + pos_bound(j) + self->cfg_.q_min(j);
  }
  for (int j = 0; j < self->dof_; ++j) {
    result[idx++] = vel_bound(j) - self->cfg_.q_dot_max(j);
  }
  for (int j = 0; j < self->dof_; ++j) {
    result[idx++] = acc_bound(j) - self->cfg_.q_ddot_max(j);
  }
}

// ---------------------------------------------------------------------------
// 可行初始点
// ---------------------------------------------------------------------------

Eigen::VectorXd
ExcitationTrajectoryOptimizer::feasibleInit() const {
  // a=0, b=0, c=q_init 严格满足所有等式约束
  // q(0) = c - sum(b/(l*wf)) = c = q_init
  // qd(0) = sum(a) = 0
  // qdd(0) = sum(b*l*wf) = 0
  // 同时 c=q_init 通常在位置范围内，满足不等式约束
  FourierTrajectoryParams params;
  params.dof = dof_;
  params.order = cfg_.fourier_order;
  params.wf = cfg_.wf;
  params.joints.resize(dof_);

  for (int j = 0; j < dof_; ++j) {
    params.joints[j].a.setZero();
    params.joints[j].b.setZero();
    params.joints[j].c = cfg_.q_init(j);
  }

  return params.toVector();
}

// ---------------------------------------------------------------------------
// 两阶段优化: COBYLA (粗搜) → SLSQP (精化)
// ---------------------------------------------------------------------------

ExcitationTrajectoryResult
ExcitationTrajectoryOptimizer::runSingle(const Eigen::VectorXd &x0,
                                          int restart_index) {
  const int n_con_eq = 3 * dof_;   // 等式约束数: q(0)=q_init, qd(0)=0, qdd(0)=0
  const int n_con_ineq = 4 * dof_; // 不等式: pos_upper, pos_lower, vel, acc

  std::vector<double> x(n_vars_);
  Eigen::VectorXd::Map(x.data(), n_vars_) = x0;
  double min_f = 0.0;
  int phase1_evals = 0;

  // ---- 阶段 1: COBYLA 粗搜 (无导数，处理约束) ----
  // 分配 80% 的评估预算给 COBYLA（至少 500）
  int phase1_maxeval = std::max(500, cfg_.max_iterations * 4 / 5);
  {
    nlopt::opt opt1(nlopt::LN_COBYLA, static_cast<unsigned>(n_vars_));
    opt1.set_min_objective(objective, this);

    std::vector<double> tol_eq(n_con_eq, 1e-8);
    opt1.add_equality_mconstraint(eqConstraints, this, tol_eq);

    std::vector<double> tol_ineq(n_con_ineq, 1e-8);
    opt1.add_inequality_mconstraint(ineqConstraints, this, tol_ineq);

    opt1.set_ftol_rel(std::max(cfg_.ftol_rel, 1e-6)); // COBYLA 宽松收敛
    opt1.set_maxeval(phase1_maxeval);
    opt1.set_xtol_rel(1e-8);

    std::vector<double> step(n_vars_, 0.1);
    opt1.set_initial_step(step);

    try {
      opt1.optimize(x, min_f);
      phase1_evals = opt1.get_numevals();
    } catch (...) {
      phase1_evals = opt1.get_numevals();
      // COBYLA 异常不致命，继续阶段 2
    }
  }

  // ---- 阶段 2: SLSQP 精化 (从 COBYLA 解出发) ----
  int phase2_evals = 0;
  {
    nlopt::opt opt2(nlopt::LD_SLSQP, static_cast<unsigned>(n_vars_));

    opt2.set_min_objective(objective, this);

    std::vector<double> tol_eq(n_con_eq, 1e-8);
    opt2.add_equality_mconstraint(eqConstraints, this, tol_eq);

    std::vector<double> tol_ineq(n_con_ineq, 1e-8);
    opt2.add_inequality_mconstraint(ineqConstraints, this, tol_ineq);

    opt2.set_ftol_rel(cfg_.ftol_rel);
    opt2.set_maxeval(cfg_.max_iterations - phase1_evals);
    opt2.set_xtol_rel(1e-10);

    try {
      opt2.optimize(x, min_f);
    } catch (...) {
      // SLSQP 异常不致命
    }
    phase2_evals = opt2.get_numevals();
  }

  // 从最终变量构建结果
  Eigen::VectorXd x_final = Eigen::VectorXd::Map(x.data(), n_vars_);
  auto params = FourierTrajectoryParams::fromVector(x_final, dof_,
                                                     cfg_.fourier_order,
                                                     cfg_.wf);
  double cond_num = computeObjective(x_final);

  // 生成轨迹 — 用 trajectory_frequency 直接由傅里叶参数求值
  const int K_out =
      static_cast<int>(cfg_.sampling_time * cfg_.trajectory_frequency) + 1;
  const double dt_out = 1.0 / cfg_.trajectory_frequency;

  ExcitationTrajectoryResult res;
  res.params = params;
  res.log_det = cond_num; // 此时 cond_num 存储的是 -log(det(W^T W))
  res.objective_value = min_f;
  res.time.resize(K_out);
  res.q_trajectory.resize(K_out, dof_);
  res.qd_trajectory.resize(K_out, dof_);
  res.qdd_trajectory.resize(K_out, dof_);

  Eigen::VectorXd q_tmp(dof_), qd_tmp(dof_), qdd_tmp(dof_);
  for (int k = 0; k < K_out; ++k) {
    double t = static_cast<double>(k) * dt_out;
    res.time[k] = t;
    params.evaluateAll(t, q_tmp, qd_tmp, qdd_tmp);
    res.q_trajectory.row(k) = q_tmp.transpose();
    res.qd_trajectory.row(k) = qd_tmp.transpose();
    res.qdd_trajectory.row(k) = qdd_tmp.transpose();
  }

  std::cout << "  起点 " << restart_index + 1 << ": -log(det)="
            << std::setprecision(4) << cond_num << "  (COBYLA: "
            << phase1_evals << " + SLSQP: " << phase2_evals
            << " 次评估)" << std::endl;

  return res;
}

// ---------------------------------------------------------------------------
// 多起点优化
// ---------------------------------------------------------------------------

Eigen::VectorXd
ExcitationTrajectoryOptimizer::randomInit(double scale,
                                           std::mt19937 &rng) const {
  std::uniform_real_distribution<double> dist(-scale, scale);
  Eigen::VectorXd x0(n_vars_);
  for (int i = 0; i < n_vars_; ++i) {
    x0(i) = dist(rng);
  }
  return x0;
}

ExcitationTrajectoryResult ExcitationTrajectoryOptimizer::optimize() {
  std::mt19937 rng(42); // 固定种子，可复现
  ExcitationTrajectoryResult best;
  best.objective_value = 1e100;

  std::cout << "\n===== 开始多起点优化 (" << cfg_.multi_start_count
            << " 个起点) =====" << std::endl;

  for (int i = 0; i < cfg_.multi_start_count; ++i) {
    // COBYLA 可处理不可行初始点 — 全部使用随机起点
    Eigen::VectorXd x0 = randomInit(0.5, rng);
    auto result = runSingle(x0, i);

    if (result.objective_value < best.objective_value) {
      best = result;
    }
  }

  std::cout << "\n===== 优化完成 =====" << std::endl;
  std::cout << "最优 D-最优值 (-log det): " << best.log_det << std::endl;

  // 验证约束
  Eigen::VectorXd q0(dof_), qd0(dof_), qdd0(dof_);
  best.params.evaluateAtZero(q0, qd0, qdd0);
  std::cout << "q(0) 约束最大偏差: " << (q0 - cfg_.q_init).cwiseAbs().maxCoeff()
            << " rad" << std::endl;
  std::cout << "qd(0) 最大偏差: " << qd0.cwiseAbs().maxCoeff() << " rad/s"
            << std::endl;
  std::cout << "qdd(0) 最大偏差: " << qdd0.cwiseAbs().maxCoeff() << " rad/s^2"
            << std::endl;

  // 稠密采样验证运动范围 (用 trajectory_frequency)
  {
    const int K_dense =
        static_cast<int>(cfg_.sampling_time * cfg_.trajectory_frequency) + 1;
    const double dt_dense = 1.0 / cfg_.trajectory_frequency;
    double max_q = -1e10, min_q = 1e10;
    double max_qd = -1e10, max_qdd = -1e10;

    Eigen::VectorXd q_tmp(dof_), qd_tmp(dof_), qdd_tmp(dof_);
    for (int k = 0; k < K_dense; ++k) {
      double t = k * dt_dense;
      best.params.evaluateAll(t, q_tmp, qd_tmp, qdd_tmp);
      max_q = std::max(max_q, q_tmp.maxCoeff());
      min_q = std::min(min_q, q_tmp.minCoeff());
      max_qd = std::max(max_qd, qd_tmp.cwiseAbs().maxCoeff());
      max_qdd = std::max(max_qdd, qdd_tmp.cwiseAbs().maxCoeff());
    }
    std::cout << "位置范围: [" << min_q << ", " << max_q << "] rad"
              << std::endl;
    std::cout << "最大速度: " << max_qd << " rad/s  (限幅: "
              << cfg_.q_dot_max.maxCoeff() << ")" << std::endl;
    std::cout << "最大加速度: " << max_qdd << " rad/s^2  (限幅: "
              << cfg_.q_ddot_max.maxCoeff() << ")" << std::endl;
  }

  return best;
}

// ---------------------------------------------------------------------------
// CSV 保存
// ---------------------------------------------------------------------------

void ExcitationTrajectoryOptimizer::saveTrajectoryCSV(
    const ExcitationTrajectoryResult &result, const std::string &path) const {
  std::ofstream out(path);
  if (!out) {
    std::cerr << "错误: 无法写入文件 " << path << std::endl;
    return;
  }

  // 表头
  out << "time";
  for (int j = 0; j < dof_; ++j)
    out << ",q" << j;
  for (int j = 0; j < dof_; ++j)
    out << ",qd" << j;
  for (int j = 0; j < dof_; ++j)
    out << ",qdd" << j;
  for (int j = 0; j < dof_; ++j)
    out << ",tau" << j;
  out << "\n";

  // 数据行 (tau 填 0)
  out << std::scientific << std::setprecision(15);
  const int K = static_cast<int>(result.time.size());
  for (int k = 0; k < K; ++k) {
    out << result.time[k];
    for (int j = 0; j < dof_; ++j)
      out << "," << result.q_trajectory(k, j);
    for (int j = 0; j < dof_; ++j)
      out << "," << result.qd_trajectory(k, j);
    for (int j = 0; j < dof_; ++j)
      out << "," << result.qdd_trajectory(k, j);
    for (int j = 0; j < dof_; ++j)
      out << "," << 0.0;
    out << "\n";
  }

  std::cout << "轨迹已保存到: " << path << " (" << K << " 行)" << std::endl;
}

// ============================================================================
// YAML 配置加载
// ============================================================================

ExcitationTrajectoryConfig
loadExciteTrajectoryConfig(const std::string &path) {
  YAML::Node root = YAML::LoadFile(path);

  ExcitationTrajectoryConfig cfg;

  // ---- 必填字段 ----
  cfg.robot_name = root["robot"].as<std::string>();
  cfg.kinematic_params_path = root["kinematic_params"].as<std::string>();

  // ---- 傅里叶参数 ----
  if (root["fourier_order"])
    cfg.fourier_order = root["fourier_order"].as<int>();
  if (root["sampling_time"])
    cfg.sampling_time = root["sampling_time"].as<double>();
  if (root["sampling_frequency"])
    cfg.sampling_frequency = root["sampling_frequency"].as<int>();
  if (root["trajectory_frequency"])
    cfg.trajectory_frequency = root["trajectory_frequency"].as<int>();
  cfg.wf = 2.0 * M_PI / cfg.sampling_time;
  cfg.num_timesteps =
      static_cast<int>(cfg.sampling_time * cfg.sampling_frequency) + 1;

  // ---- 关节限位 ----
  cfg.q_init = parseVectorXd(root["q_init"]);
  cfg.q_min = parseVectorXd(root["q_min"]);
  cfg.q_max = parseVectorXd(root["q_max"]);
  cfg.q_dot_max = parseVectorXd(root["q_dot_max"]);
  cfg.q_ddot_max = parseVectorXd(root["q_ddot_max"]);

  // ---- 一致性检查 ----
  const int D = static_cast<int>(cfg.q_init.size());
  if (static_cast<int>(cfg.q_min.size()) != D ||
      static_cast<int>(cfg.q_max.size()) != D ||
      static_cast<int>(cfg.q_dot_max.size()) != D ||
      static_cast<int>(cfg.q_ddot_max.size()) != D) {
    throw std::runtime_error(
        "excite_trajectory.yaml: q_init / q_min / q_max / q_dot_max / "
        "q_ddot_max 维度不一致");
  }

  // ---- 优化参数 (可选，有默认值) ----
  if (root["regularization_lambda"])
    cfg.regularization_lambda = root["regularization_lambda"].as<double>();
  if (root["multi_start_count"])
    cfg.multi_start_count = root["multi_start_count"].as<int>();
  if (root["max_iterations"])
    cfg.max_iterations = root["max_iterations"].as<int>();
  if (root["ftol_rel"])
    cfg.ftol_rel = root["ftol_rel"].as<double>();

  // ---- 输出路径 ----
  if (root["output_trajectory"])
    cfg.output_trajectory_path = root["output_trajectory"].as<std::string>();

  return cfg;
}

} // namespace excitation_trajectory
