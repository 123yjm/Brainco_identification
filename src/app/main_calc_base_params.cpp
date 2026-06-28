/**
 * @file main_calc_base_params.cpp
 * @brief 随机采样 → 构建 W → 列缩放 QRCP → 输出基参数集
 *
 * 参考 robotdynid: 在关节限位内随机采样 q/qd/qdd，构建观测矩阵 W，
 * 通过 QR 列主元分解获取最小惯性参数集（base parameter set）。
 *
 * 用法: ./calc_base_params --robot <robot_dir> [--samples 800] [--tol 100]
 */
#include "base_parameter.hpp"
#include "regressor_factory.hpp"
#include "robot_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

void printSep() { std::cout << std::string(60, '-') << "\n"; }

void printHelp(const char *prog) {
  std::cout << "基参数集计算 — 随机采样 + QR 列主元分解\n\n"
            << "用法: " << prog << " --robot <robot_dir> [选项]\n\n"
            << "选项:\n"
            << "  --robot <dir>     机器人目录\n"
            << "  --samples <N>     随机采样数 (默认 800)\n"
            << "  --tol <value>     QRCP 秩阈值乘数 (默认 100)\n"
            << "  --seed <value>    随机种子 (默认 42)\n"
            << "  --help            打印帮助\n";
}

Eigen::VectorXd parseVec(const YAML::Node &node) {
  Eigen::VectorXd v(node.size());
  for (std::size_t i = 0; i < node.size(); ++i)
    v(i) = node[i].as<double>();
  return v;
}

} // namespace

int main(int argc, char *argv[]) {
  std::string robot_dir;
  int n_samples = 800;
  double tol = 100.0;
  int seed = 42;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help") { printHelp(argv[0]); return 0; }
    else if ((arg == "--robot" || arg == "-r") && i + 1 < argc)
      robot_dir = robot_utils::resolvePath(
          robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
    else if (arg == "--samples" && i + 1 < argc)
      n_samples = std::stoi(argv[++i]);
    else if (arg == "--tol" && i + 1 < argc)
      tol = std::stod(argv[++i]);
    else if (arg == "--seed" && i + 1 < argc)
      seed = std::stoi(argv[++i]);
  }
  if (robot_dir.empty()) { std::cerr << "需要 --robot\n"; return 1; }

  std::string robot_name = robot_utils::robotNameFromDir(robot_dir);
  std::string kinematic_yaml = robot_utils::configPath(robot_dir, "kinematic_params.yaml");
  std::string excite_yaml = robot_utils::configPath(robot_dir, "excite_trajectory.yaml");
  std::string output_yaml = robot_utils::resultBaseInertiaPath(robot_dir,
      robot_name + "_base_params_metadata.yaml");

  // ---- 加载限位 -------------------------------------------------------------
  Eigen::VectorXd q_min, q_max, qd_max, qdd_max;
  try {
    auto root = YAML::LoadFile(excite_yaml);
    q_min   = parseVec(root["q_min"]);
    q_max   = parseVec(root["q_max"]);
    qd_max  = parseVec(root["q_dot_max"]);
    qdd_max = parseVec(root["q_ddot_max"]);
  } catch (const std::exception &e) {
    std::cerr << "无法加载限位: " << e.what() << "\n";
    return 1;
  }

  // ---- 创建 regressor -------------------------------------------------------
  std::string robot_type = robot_name;
  try {
    auto kroot = YAML::LoadFile(kinematic_yaml);
    if (kroot["robot_type"]) robot_type = kroot["robot_type"].as<std::string>();
  } catch (...) {}

  std::unique_ptr<robot_dynamics::IDynamicsRegressor> reg;
  try {
    reg = robot_dynamics::RegressorFactory::create(robot_type, kinematic_yaml);
  } catch (const std::exception &e) {
    std::cerr << "无法创建 regressor: " << e.what() << "\n";
    return 1;
  }

  const std::size_t n_dof = reg->nDof();
  const std::size_t n_bodies = reg->nBodies();
  auto flags = robot_dynamics::ParamFlags::NONE; // 仅惯性参数
  const std::size_t n_params = reg->numParameters(flags);

  std::cout << "机器人: " << robot_name << "\n"
            << "DOF: " << n_dof << ", Bodies: " << n_bodies
            << ", 惯性参数: " << n_params << "\n"
            << "采样数: " << n_samples << ", tol: " << tol
            << ", seed: " << seed << "\n\n";

  // ---- 随机采样 -------------------------------------------------------------
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  // 速度/加速度缩放 (参考 robotdynid: velocity_scale=0.5, accel_scale=0.5)
  const double vel_scale = 0.5;
  const double acc_scale = 0.5;

  const int K = n_samples;
  Eigen::MatrixXd Q(K, static_cast<Eigen::Index>(n_dof));
  Eigen::MatrixXd Qd(K, static_cast<Eigen::Index>(n_dof));
  Eigen::MatrixXd Qdd(K, static_cast<Eigen::Index>(n_dof));

  for (int k = 0; k < K; ++k) {
    for (std::size_t j = 0; j < n_dof; ++j) {
      Q(k, j) = q_min(j) + dist(rng) * (q_max(j) - q_min(j));
      double vlim = vel_scale * qd_max(j);
      Qd(k, j) = -vlim + 2.0 * vlim * dist(rng);
      double alim = acc_scale * qdd_max(j);
      Qdd(k, j) = -alim + 2.0 * alim * dist(rng);
    }
  }

  // ---- 构建 W ---------------------------------------------------------------
  std::cout << "构建观测矩阵 W (" << (K * n_dof) << " × " << n_params << ") ...\n";
  Eigen::MatrixXd W(n_dof * K, n_params);
  for (int k = 0; k < K; ++k) {
    Eigen::VectorXd q   = Q.row(k).transpose();
    Eigen::VectorXd qd  = Qd.row(k).transpose();
    Eigen::VectorXd qdd = Qdd.row(k).transpose();
    Eigen::MatrixXd Y_k = reg->computeRegressorMatrix(q, qd, qdd, flags);
    W.block(k * static_cast<Eigen::Index>(n_dof), 0,
            static_cast<Eigen::Index>(n_dof),
            static_cast<Eigen::Index>(n_params)) = Y_k;
  }

  // ---- QRCP 分解 (列缩放 + tol=100) -----------------------------------------
  std::cout << "QRCP 分解 (列缩放, tol=" << tol << ") ...\n";
  auto dec = robot_dynamics::BaseParameterDecomposition::compute(W, tol);

  printSep();
  std::cout << "数值秩:     " << dec.rank << " / " << dec.n_full << "\n"
            << "秩亏:       " << (dec.n_full - dec.rank) << "\n"
            << "条件数 R1:  " << std::scientific << dec.cond_R1 << std::fixed << "\n"
            << "阈值:       " << std::scientific << dec.threshold << std::fixed << "\n";

  // ---- 输出 ----------------------------------------------------------------
  std::vector<std::string> full_names = reg->getParameterNames(flags);
  std::cout << "\n可辨识参数 (" << dec.rank << "):\n";
  for (std::size_t i = 0; i < dec.identifiable_indices.size(); ++i) {
    Eigen::Index idx = dec.identifiable_indices[i];
    std::cout << "  [" << idx << "] " << full_names[idx] << "\n";
  }
  std::cout << "\n不可辨识参数 (" << (dec.n_full - dec.rank) << "):\n";
  for (Eigen::Index i = 0; i < dec.n_full - dec.rank; ++i) {
    Eigen::Index idx = dec.unidentifiable_indices[static_cast<std::size_t>(i)];
    std::cout << "  [" << idx << "] " << full_names[idx] << "\n";
  }

  // ---- 保存 YAML -----------------------------------------------------------
  std::filesystem::create_directories(
      robot_utils::resultBaseInertiaPath(robot_dir, ""));
  std::ofstream out(output_yaml);
  out << std::scientific << std::setprecision(8);
  out << "calibration_date: \""; {
    std::time_t n = std::time(nullptr);
    char b[64]; std::strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", std::localtime(&n));
    out << b;
  }
  out << "\"\nrobot: \"" << robot_name << "\"\n"
      << "method: \"random_sampling_qrcp\"\n"
      << "samples: " << n_samples << "\n"
      << "random_seed: " << seed << "\n"
      << "rank_tol_factor: " << tol << "\n"
      << "column_scaling: true\n"
      << "decomposition:\n"
      << "  rank: " << dec.rank << "\n"
      << "  rank_deficiency: " << (dec.n_full - dec.rank) << "\n"
      << "  parameters_full: " << dec.n_full << "\n"
      << "  condition_number_R1: " << dec.cond_R1 << "\n"
      << "  threshold: " << dec.threshold << "\n"
      << "identifiable_parameters:\n"
      << "  count: " << dec.rank << "\n"
      << "  indices: [";
  for (std::size_t i = 0; i < dec.identifiable_indices.size(); ++i)
    out << dec.identifiable_indices[i] << (i+1 < dec.identifiable_indices.size() ? ", " : "");
  out << "]\n  names: [";
  for (std::size_t i = 0; i < dec.identifiable_indices.size(); ++i)
    out << "\"" << full_names[dec.identifiable_indices[i]] << "\""
        << (i+1 < dec.identifiable_indices.size() ? ", " : "");
  out << "]\n"
      << "unidentifiable_parameters:\n"
      << "  count: " << (dec.n_full - dec.rank) << "\n"
      << "  indices: [";
  for (Eigen::Index i = 0; i < dec.n_full - dec.rank; ++i)
    out << dec.unidentifiable_indices[static_cast<std::size_t>(i)]
        << (i+1 < dec.n_full - dec.rank ? ", " : "");
  out << "]\n  names: [";
  for (Eigen::Index i = 0; i < dec.n_full - dec.rank; ++i)
    out << "\"" << full_names[dec.unidentifiable_indices[static_cast<std::size_t>(i)]] << "\""
        << (i+1 < dec.n_full - dec.rank ? ", " : "");
  out << "]\n";
  if (dec.n_full - dec.rank > 0) {
    out << "regrouping_map:\n"
        << "  alpha_rows: " << dec.alpha.rows() << "\n"
        << "  alpha_cols: " << dec.alpha.cols() << "\n";
  }

  std::cout << "\n结果已保存: " << output_yaml << "\n";
  return 0;
}
