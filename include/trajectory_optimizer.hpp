#pragma once

/**
 * @file trajectory_optimizer.hpp
 * @brief 傅里叶激励轨迹优化
 *
 * 优化 5 阶傅里叶级数系数 (a, b, c)，使得堆叠观测矩阵 W 的条件数最小：
 *   min  cond(W^T W + lambda*I)
 *   s.t. q(0) = q_init,  qd(0) = 0,  qdd(0) = 0
 *        关节位置 / 速度 / 加速度不超限 (幅值求和上界)
 *
 * 使用 NLopt SLSQP 求解约束非线性规划，多起点随机初始化。
 */

#include "dynamics_regressor.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <cstddef>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace excitation_trajectory {

// ============================================================================
// FourierJointParams — 单关节傅里叶级数参数 (5 阶)
// ============================================================================

/// 单关节 5 阶傅里叶级数系数
///
/// q_j(t)  = c_j + Sum_{l=1..5} [ a_l * sin(l*wf*t) / (l*wf)
///                               - b_l * cos(l*wf*t) / (l*wf) ]
/// qd_j(t) = Sum_{l=1..5} [ a_l * cos(l*wf*t) + b_l * sin(l*wf*t) ]
/// qdd_j(t)= Sum_{l=1..5} [ -a_l * l*wf * sin(l*wf*t) + b_l * l*wf * cos(l*wf*t) ]
struct FourierJointParams {
  static constexpr int kOrder = 5;

  /// a 系数 (正弦项), 长度 5
  Eigen::Matrix<double, kOrder, 1> a =
      Eigen::Matrix<double, kOrder, 1>::Zero();

  /// b 系数 (余弦项), 长度 5
  Eigen::Matrix<double, kOrder, 1> b =
      Eigen::Matrix<double, kOrder, 1>::Zero();

  /// c 偏移 (直流分量)
  double c = 0.0;

  // ---- 运动学评估 ----------------------------------------------------------

  /// 在时刻 t 评估单关节 q, qd, qdd (@param wf 基频 2*pi/T)
  void evaluate(double t, double wf, double &q_out, double &qd_out,
                double &qdd_out) const;

  // ---- 幅值上界 (三角不等式保守估计) ----------------------------------------

  /// 速度幅值上界:  max|qd| <= sum_l sqrt(a_l^2 + b_l^2)
  double velocityAmplitudeBound() const;

  /// 加速度幅值上界: max|qdd| <= sum_l sqrt(a_l^2 + b_l^2) * l * wf
  double accelerationAmplitudeBound(double wf) const;

  /// 位置波动幅值上界:
  ///   max|q - c| <= sum_l sqrt(a_l^2 + b_l^2) / (l * wf)
  double positionAmplitudeBound(double wf) const;

  // ---- t=0 解析值 ----------------------------------------------------------

  /// q(0) = c - sum_l b_l / (l * wf)
  double qAtZero(double wf) const;

  /// qd(0) = sum_l a_l
  double qdAtZero() const;

  /// qdd(0) = sum_l b_l * l * wf
  double qddAtZero(double wf) const;
};

// ============================================================================
// FourierTrajectoryParams — 全关节傅里叶轨迹参数
// ============================================================================

struct FourierTrajectoryParams {
  int dof = 0;
  int order = 5;
  double wf = 0.0; ///< 基频 (rad/s)

  std::vector<FourierJointParams> joints;

  /// 从扁平优化变量向量 x 恢复结构
  ///
  /// x 布局 (匹配 MATLAB):
  ///   a: [a1_1..aD_1, a1_2..aD_2, ..., a1_N..aD_N]    (D*N 个)
  ///   b: [b1_1..bD_1, b1_2..bD_2, ..., b1_N..bD_N]    (D*N 个)
  ///   c: [c1..cD]                                      (D   个)
  static FourierTrajectoryParams
  fromVector(const Eigen::VectorXd &x, int dof, int order, double wf);

  /// 逆操作：展平为优化变量向量
  Eigen::VectorXd toVector() const;

  /// 在时刻 t 评估全部关节运动学
  void evaluateAll(double t, Eigen::Ref<Eigen::VectorXd> q,
                   Eigen::Ref<Eigen::VectorXd> qd,
                   Eigen::Ref<Eigen::VectorXd> qdd) const;

  /// 各关节速度幅值上界 (dof 维)
  Eigen::VectorXd velocityAmplitudeBound() const;

  /// 各关节加速度幅值上界 (dof 维)
  Eigen::VectorXd accelerationAmplitudeBound() const;

  /// 各关节位置波动幅值上界 (dof 维)
  Eigen::VectorXd positionAmplitudeBound() const;

  /// t=0 时刻全部关节 q, qd, qdd
  void evaluateAtZero(Eigen::Ref<Eigen::VectorXd> q0,
                      Eigen::Ref<Eigen::VectorXd> qd0,
                      Eigen::Ref<Eigen::VectorXd> qdd0) const;
};

// ============================================================================
// ExcitationTrajectoryConfig — 从 YAML 加载的优化配置
// ============================================================================

struct ExcitationTrajectoryConfig {
  std::string robot_name;
  std::string kinematic_params_path;

  int fourier_order = 5;
  double sampling_time = 10.0;        ///< 激励轨迹运行时长 (秒)
  int sampling_frequency = 10;        ///< 优化阶段采样频率 (Hz)，构建 W 用
  int trajectory_frequency = 100;     ///< 轨迹发布频率 (Hz)，最终输出用
  double wf = 0.0;                    ///< 基频 = 2*pi/sampling_time
  int num_timesteps = 101;            ///< 优化采样点 = sampling_time * sampling_frequency + 1

  Eigen::VectorXd q_init;     ///< dof × 1  起始/终止关节角
  Eigen::VectorXd q_min;      ///< dof × 1  位置下限
  Eigen::VectorXd q_max;      ///< dof × 1  位置上限
  Eigen::VectorXd q_dot_max;  ///< dof × 1  速度幅值上限
  Eigen::VectorXd q_ddot_max; ///< dof × 1  加速度幅值上限

  double regularization_lambda = 1e-6;
  int multi_start_count = 8;
  int max_iterations = 15000;
  double ftol_rel = 1e-10;

  std::string output_trajectory_path;
};

// ============================================================================
// ExcitationTrajectoryResult
// ============================================================================

struct ExcitationTrajectoryResult {
  FourierTrajectoryParams params;
  double log_det = 0.0;       ///< -log(det(W^T W))
  double objective_value = 0.0;

  /// 最优轨迹 (K × dof 行优先，每行一个采样点)
  Eigen::MatrixXd q_trajectory;
  Eigen::MatrixXd qd_trajectory;
  Eigen::MatrixXd qdd_trajectory;
  std::vector<double> time;
};

// ============================================================================
// ExcitationTrajectoryOptimizer
// ============================================================================

class ExcitationTrajectoryOptimizer {
public:
  /// @param cfg 优化配置 (含关节限位、初始角等)
  explicit ExcitationTrajectoryOptimizer(const ExcitationTrajectoryConfig &cfg);

  /// 运行多起点优化，返回最优结果
  ExcitationTrajectoryResult optimize();

  /// 保存轨迹为 CSV
  void saveTrajectoryCSV(const ExcitationTrajectoryResult &result,
                         const std::string &path) const;

private:
  ExcitationTrajectoryConfig cfg_;
  std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor_;
  int n_vars_; ///< 优化变量总数 = dof * (2*order + 1)
  int dof_;
  int num_params_;

  // ---- NLopt 回调 (C 风格 static 函数，通过 void* 访问 this) ---------------

  /// 目标函数: f(x) = cond(W(x)^T * W(x) + lambda * I)
  static double objective(unsigned n, const double *x, double *grad,
                          void *data);

  /// 等式约束 (3*dof 个): q(0)=q_init, qd(0)=0, qdd(0)=0
  static void eqConstraints(unsigned m, double *result, unsigned n,
                            const double *x, double *grad, void *data);

  /// 不等式约束 (4*dof 个): 位置上下界/速度/加速度幅值不超限
  static void ineqConstraints(unsigned m, double *result, unsigned n,
                              const double *x, double *grad, void *data);

  // ---- 内部实现 ------------------------------------------------------------

  /// 从优化变量 x 构建 W，返回 cond(W^T W + lambda*I)
  double computeObjective(const Eigen::VectorXd &x) const;

  /// 单次 AUGLAG+SLSQP 运行
  ExcitationTrajectoryResult runSingle(const Eigen::VectorXd &x0,
                                       int restart_index);

  /// 随机初始点 (均匀分布 [-scale, scale])
  Eigen::VectorXd randomInit(double scale, std::mt19937 &rng) const;

  /// 可行初始点 (满足全部等式约束: a=0, b=0, c=q_init)
  Eigen::VectorXd feasibleInit() const;
};

// ============================================================================
// YAML 加载
// ============================================================================

ExcitationTrajectoryConfig
loadExciteTrajectoryConfig(const std::string &path);

} // namespace excitation_trajectory
