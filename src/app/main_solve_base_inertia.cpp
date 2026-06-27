/**
 * @file main_solve_base_inertia.cpp
 * @brief 最小惯性参数集辨识 — QR 列主元分解 + 摩擦力扣除
 *
 * 用法: ./identify_base_inertia --robot <robot_dir> [选项]
 *
 * 流程:
 *   1. 加载滤波数据
 *   2. 加载已辨识的摩擦参数 (Fc, Fv)
 *   3. τ' = τ - Fc·sign(q̇) - Fv·q̇
 *   4. 构建纯惯性观测矩阵 W
 *   5. QRCP 分解 → 找可辨识子空间
 *   6. 在最小参数集上 OLS 求解
 *   7. 映射回全参数空间并输出
 */

#include "algorithms.hpp"
#include "base_parameter.hpp"
#include "data_loader.hpp"
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
#include <sstream>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 帮助信息
// ---------------------------------------------------------------------------
void printHelp(const char *prog) {
  std::cout
      << "最小惯性参数集辨识 — QR 列主元分解\n\n"
      << "用法: " << prog << " --robot <robot_dir> [选项]\n\n"
      << "选项:\n"
      << "  --robot <dir>         机器人目录 (如 robots/revoarm_right)\n"
      << "  --algo <name>         求解算法 (默认 OLS)\n"
      << "  --no-friction-sub     不扣除已辨识的摩擦力\n"
      << "  --armature            包含电机转子惯量 (默认不包含)\n"
      << "  --tol <value>         秩判定阈值乘数 (默认 1.0)\n"
      << "  --help                打印帮助信息\n\n"
      << "说明:\n"
      << "  - 默认对纯惯性参数 (n_bodies×10) 做 QR 列主元分解\n"
      << "  - 默认从 result_friction/*_friction_identification.yaml 加载摩擦参数\n"
      << "  - 最小参数集 β_base 满足 τ' = W_base · β_base (列满秩)\n"
      << "  - 结果映射回全参数空间后输出，不可辨识参数设为零\n";
}

// ---------------------------------------------------------------------------
// 输出分隔线
// ---------------------------------------------------------------------------
void printSep() { std::cout << std::string(60, '-') << "\n"; }

// ---------------------------------------------------------------------------
// 从 YAML node 解析 double 向量
// ---------------------------------------------------------------------------
Eigen::VectorXd parseDoubleVector(const YAML::Node &node) {
  Eigen::VectorXd v(node.size());
  for (std::size_t i = 0; i < node.size(); ++i)
    v(static_cast<Eigen::Index>(i)) = node[i].as<double>();
  return v;
}

// ---------------------------------------------------------------------------
// 加载摩擦参数
// ---------------------------------------------------------------------------
bool loadFrictionParams(const std::string &yaml_path,
                        std::size_t dof,
                        Eigen::VectorXd &fc,
                        Eigen::VectorXd &fv) {
  try {
    auto root = YAML::LoadFile(yaml_path);
    auto params_node = root["parameters"];
    if (!params_node || !params_node.IsSequence()) {
      std::cerr << "  警告: YAML 中无 'parameters' 序列\n";
      return false;
    }
    // friction YAML 格式: parameters: [Fc0, Fv0, Fc1, Fv1, ...]
    // 每个元素是标量，按顺序配对
    std::size_t n_vals = params_node.size();
    fc.resize(dof);
    fv.resize(dof);
    for (std::size_t j = 0; j < dof && 2 * j + 1 < n_vals; ++j) {
      fc(static_cast<Eigen::Index>(j)) = params_node[2 * j].as<double>();
      fv(static_cast<Eigen::Index>(j)) = params_node[2 * j + 1].as<double>();
    }
    return true;
  } catch (const std::exception &e) {
    std::cerr << "  警告: 加载摩擦参数失败 — " << e.what() << "\n";
    return false;
  }
}

// ---------------------------------------------------------------------------
// 保存结果为 YAML
// ---------------------------------------------------------------------------
void saveBaseResults(
    const std::string &path,
    const std::string &robot_name,
    const std::string &algo_name,
    const std::string &friction_source,
    bool friction_subtracted,
    bool armature_included,
    const robot_dynamics::BaseParameterDecomposition::Decomposition &dec,
    const Eigen::VectorXd &beta_base,
    const Eigen::VectorXd &beta_full,
    double rmse_base,
    double max_err_base,
    double rmse_full,
    double max_err_full,
    std::size_t n_bodies,
    const std::vector<std::string> &full_param_names,
    const std::vector<std::string> &formulas,
    const Eigen::MatrixXd &C_matrix) {

  std::filesystem::path p(path);
  if (p.has_parent_path())
    std::filesystem::create_directories(p.parent_path());

  std::ofstream out(path);
  if (!out) {
    std::cerr << "无法写入: " << path << std::endl;
    return;
  }

  std::time_t now = std::time(nullptr);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

  out << std::scientific << std::setprecision(8);

  // ---- 头部 ----
  out << "calibration_date: \"" << buf << "\"\n"
      << "robot: \"" << robot_name << "\"\n"
      << "method: \"qr_decomposition_base_parameters\"\n"
      << "friction_source: \"" << friction_source << "\"\n"
      << "friction_subtracted: " << (friction_subtracted ? "true" : "false") << "\n"
      << "armature_included: " << (armature_included ? "true" : "false") << "\n"
      << "damping_included: false\n\n";

  // ---- 分解信息 ----
  out << "decomposition:\n"
      << "  rank: " << dec.rank << "\n"
      << "  rank_deficiency: " << (dec.n_full - dec.rank) << "\n"
      << "  parameters_full: " << dec.n_full << "\n"
      << "  condition_number_R1: " << dec.cond_R1 << "\n"
      << "  threshold: " << dec.threshold << "\n\n";

  // ---- 基参数解 ----
  out << "base_solution:\n"
      << "  algorithm: \"" << algo_name << "\"\n"
      << "  torque_rmse: " << rmse_base << "\n"
      << "  torque_max_error: " << max_err_base << "\n"
      << "  base_parameters: [";
  for (Eigen::Index i = 0; i < beta_base.size(); ++i) {
    out << beta_base(i);
    if (i + 1 < beta_base.size()) out << ", ";
  }
  out << "]\n\n";

  // ---- 全参数重建 ----
  out << "full_reconstruction:\n"
      << "  torque_rmse: " << rmse_full << "\n"
      << "  torque_max_error: " << max_err_full << "\n"
      << "  inertia:\n";

  Eigen::Index n_body_idx = static_cast<Eigen::Index>(n_bodies);
  for (Eigen::Index b = 0; b < n_body_idx; ++b) {
    out << "    - [";
    Eigen::Index base = b * 10;
    for (int j = 0; j < 10; ++j) {
      out << beta_full(base + j);
      if (j < 9) out << ", ";
    }
    out << "]\n";
  }

  // armature (如果有)
  Eigen::Index inertial_total = n_body_idx * 10;
  if (beta_full.size() > inertial_total) {
    out << "  armature: [";
    for (Eigen::Index j = inertial_total; j < beta_full.size(); ++j) {
      out << beta_full(j);
      if (j + 1 < beta_full.size()) out << ", ";
    }
    out << "]\n";
  }

  // ---- 可辨识参数列表 ----
  out << "\nidentifiable_parameters:\n"
      << "  count: " << dec.rank << "\n"
      << "  indices: [";
  for (std::size_t i = 0; i < dec.identifiable_indices.size(); ++i) {
    out << dec.identifiable_indices[i];
    if (i + 1 < dec.identifiable_indices.size()) out << ", ";
  }
  out << "]\n"
      << "  names: [";
  for (std::size_t i = 0; i < dec.identifiable_indices.size(); ++i) {
    Eigen::Index idx = dec.identifiable_indices[i];
    if (idx < static_cast<Eigen::Index>(full_param_names.size()))
      out << "\"" << full_param_names[static_cast<std::size_t>(idx)] << "\"";
    else
      out << "\"col_" << idx << "\"";
    if (i + 1 < dec.identifiable_indices.size()) out << ", ";
  }
  out << "]\n\n";

  // ---- 不可辨识参数列表 ----
  Eigen::Index deficiency = dec.n_full - dec.rank;
  out << "unidentifiable_parameters:\n"
      << "  count: " << deficiency << "\n"
      << "  indices: [";
  for (Eigen::Index i = 0; i < deficiency; ++i) {
    out << dec.unidentifiable_indices[static_cast<std::size_t>(i)];
    if (i + 1 < deficiency) out << ", ";
  }
  out << "]\n"
      << "  names: [";
  for (Eigen::Index i = 0; i < deficiency; ++i) {
    Eigen::Index idx = dec.unidentifiable_indices[static_cast<std::size_t>(i)];
    if (idx < static_cast<Eigen::Index>(full_param_names.size()))
      out << "\"" << full_param_names[static_cast<std::size_t>(idx)] << "\"";
    else
      out << "\"col_" << idx << "\"";
    if (i + 1 < deficiency) out << ", ";
  }
  out << "]\n";

  // ---- 重组关系 (alpha 矩阵) ----
  if (deficiency > 0) {
    out << "\nregrouping_map:\n"
        << "  description: \"beta_base = beta_identifiable + alpha * beta_unidentifiable\"\n"
        << "  alpha_rows: " << dec.alpha.rows() << "\n"
        << "  alpha_cols: " << dec.alpha.cols() << "\n"
        << "  alpha_values: [";
    for (Eigen::Index i = 0; i < dec.alpha.rows(); ++i) {
      out << "\n    ";
      for (Eigen::Index j = 0; j < dec.alpha.cols(); ++j) {
        out << dec.alpha(i, j);
        if (j + 1 < dec.alpha.cols() || i + 1 < dec.alpha.rows()) out << ", ";
      }
    }
    out << "\n  ]\n";
  }

  // ---- 组合矩阵 (C: β_base = C · β_full) ----
  if (!formulas.empty()) {
    out << "\ncombination_matrix:\n"
        << "  description: \"β_base (r×1) = C (r×n) × β_full (n×1). "
        << "代入 CAD 先验值计算基参数预测值.\"\n"
        << "  rows: " << C_matrix.rows() << "\n"
        << "  cols: " << C_matrix.cols() << "\n"
        << "  formulas:\n";
    for (const auto &f : formulas) {
      out << "    - \"" << f << "\"\n";
    }

    // C 矩阵按行展平
    out << "  matrix_C: [";
    for (Eigen::Index i = 0; i < C_matrix.rows(); ++i) {
      out << "\n    ";
      for (Eigen::Index j = 0; j < C_matrix.cols(); ++j) {
        out << C_matrix(i, j);
        if (j + 1 < C_matrix.cols() || i + 1 < C_matrix.rows()) out << ", ";
      }
    }
    out << "\n  ]\n";
  }

  std::cout << "结果已保存: " << path << std::endl;
}

} // anonymous namespace

// ============================================================================
// main
// ============================================================================
int main(int argc, char *argv[]) {
  // ---- CLI 解析 -----------------------------------------------------------
  std::string robot_dir;
  std::string algo_name = "OLS";
  bool friction_sub = true;
  bool armature = false;
  double tol = 1.0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help") {
      printHelp(argv[0]);
      return 0;
    } else if ((arg == "--robot" || arg == "-r") && i + 1 < argc) {
      robot_dir = robot_utils::resolvePath(
          robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
    } else if (arg == "--algo" && i + 1 < argc) {
      algo_name = argv[++i];
    } else if (arg == "--no-friction-sub") {
      friction_sub = false;
    } else if (arg == "--armature") {
      armature = true;
    } else if (arg == "--tol" && i + 1 < argc) {
      tol = std::stod(argv[++i]);
      if (tol <= 0.0) {
        std::cerr << "错误: --tol 必须 > 0\n";
        return 1;
      }
    }
  }

  if (robot_dir.empty()) {
    std::cerr << "错误: 需要 --robot <dir>\n";
    return 1;
  }

  // ---- 派生路径 -----------------------------------------------------------
  std::string robot_name = robot_utils::robotNameFromDir(robot_dir);
  std::string kinematic_yaml =
      robot_utils::configPath(robot_dir, "kinematic_params.yaml");
  std::string data_file =
      robot_utils::resultInertiaPath(robot_dir, robot_name + "_filtered_data.csv");
  std::string friction_yaml =
      robot_utils::resultFrictionPath(robot_dir, robot_name + "_friction_identification.yaml");
  std::string output_file =
      robot_utils::resultInertiaPath(robot_dir, robot_name + "_base_inertia_identification.yaml");

  // ---- robot_type ---------------------------------------------------------
  std::string robot_type = robot_name;
  try {
    auto kroot = YAML::LoadFile(kinematic_yaml);
    if (kroot["robot_type"])
      robot_type = kroot["robot_type"].as<std::string>();
  } catch (const std::exception &e) {
    std::cerr << "警告: kinematic_params.yaml — " << e.what() << "\n";
  }

  // ---- 创建 regressor -----------------------------------------------------
  std::cout << "机器人: " << robot_name << " (type: " << robot_type << ")"
            << "\n运动学: " << kinematic_yaml
            << "\n数据:   " << data_file << std::endl;

  std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor;
  try {
    regressor = robot_dynamics::RegressorFactory::create(robot_type, kinematic_yaml);
  } catch (const std::exception &e) {
    std::cerr << "错误: " << e.what() << std::endl;
    return 1;
  }

  const std::size_t n_dof = regressor->nDof();
  const std::size_t n_bodies = regressor->nBodies();

  // ---- 加载数据 -----------------------------------------------------------
  ExperimentData data;
  try {
    data = DataLoader::loadCSV(data_file, n_dof);
  } catch (const std::exception &e) {
    std::cerr << "错误: 数据加载失败 — " << e.what() << std::endl;
    return 1;
  }
  std::cout << "样本数: " << data.n_samples << ", DOF: " << n_dof
            << ", Bodies: " << n_bodies << std::endl;

  // ---- ParamFlags ---------------------------------------------------------
  auto flags = robot_dynamics::ParamFlags::NONE;
  if (armature)
    flags = flags | robot_dynamics::ParamFlags::ARMATURE;
  // damping 始终 OFF — 摩擦已单独辨识

  const std::size_t num_params = regressor->numParameters(flags);
  std::cout << "全参数数: " << num_params
            << " (惯性=" << (n_bodies * 10) << ")";
  if (armature)
    std::cout << " + armature=" << n_dof;
  std::cout << "\n";

  // ---- 加载摩擦参数 -------------------------------------------------------
  Eigen::VectorXd fc, fv;
  bool friction_loaded = false;
  if (friction_sub) {
    friction_loaded = loadFrictionParams(friction_yaml, n_dof, fc, fv);
    if (friction_loaded) {
      std::cout << "摩擦参数: " << friction_yaml << "\n";
      for (std::size_t j = 0; j < n_dof; ++j) {
        std::cout << "  j" << j << ": Fc=" << fc(j) << ", Fv=" << fv(j) << "\n";
      }
    } else if (friction_sub) {
      std::cout << "警告: 摩擦参数加载失败，跳过摩擦扣除\n";
    }
  } else {
    std::cout << "摩擦扣除: OFF (--no-friction-sub)\n";
  }

  // ---- 构建观测矩阵 W + 扣除摩擦 ------------------------------------------
  const Eigen::Index K = static_cast<Eigen::Index>(data.n_samples);
  const Eigen::Index rows_per = static_cast<Eigen::Index>(n_dof);
  const Eigen::Index n_params = static_cast<Eigen::Index>(num_params);
  const Eigen::Index n_rows = rows_per * K;

  std::cout << "构造观测矩阵 W (" << n_rows << " × " << n_params << ") ...\n";

  Eigen::MatrixXd W_full = Eigen::MatrixXd::Zero(n_rows, n_params);
  Eigen::VectorXd tau_stacked(n_rows);

#ifdef IDENTIFICATION_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (Eigen::Index k = 0; k < K; ++k) {
    Eigen::VectorXd q = data.q.row(k).transpose();
    Eigen::VectorXd qd = data.qd.row(k).transpose();
    Eigen::VectorXd qdd = data.qdd.row(k).transpose();

    Eigen::MatrixXd Y_k = regressor->computeRegressorMatrix(q, qd, qdd, flags);

    Eigen::Index row_offset = k * rows_per;
    W_full.block(row_offset, 0, rows_per, n_params) = Y_k;

    for (Eigen::Index j = 0; j < rows_per; ++j) {
      double tau_val = data.tau(k, j);

      // 扣除摩擦
      if (friction_loaded) {
        double dq = qd(j);
        double tau_fric = fc(j) * ((dq >= 0.0) ? 1.0 : -1.0) + fv(j) * dq;
        tau_val -= tau_fric;
      }

      tau_stacked(row_offset + j) = tau_val;
    }
  }

  // ---- QRCP 分解 -----------------------------------------------------------
  std::cout << "QR 列主元分解 ...\n";
  auto dec = robot_dynamics::BaseParameterDecomposition::compute(W_full, tol);

  printSep();
  std::cout << "数值秩:       " << dec.rank << " / " << dec.n_full << "\n"
            << "秩亏:         " << (dec.n_full - dec.rank) << "\n"
            << "条件数 (R1):  " << dec.cond_R1 << "\n"
            << "阈值:         " << std::scientific << dec.threshold << std::fixed
            << "\n";

  // 奇异值近似 (|R 对角线|)
  std::cout << "\n|R 对角线| 前 10:\n";
  for (Eigen::Index i = 0; i < std::min<Eigen::Index>(10, dec.R_diag.size()); ++i) {
    std::cout << "  |R(" << i << "," << i << ")| = " << std::scientific
              << dec.R_diag(i);
    if (i >= dec.rank) std::cout << "  <-- 秩亏";
    std::cout << "\n";
  }
  if (dec.R_diag.size() > 10) {
    Eigen::Index tail = std::max<Eigen::Index>(10,
        dec.R_diag.size() - std::min<Eigen::Index>(5, dec.n_full - dec.rank));
    std::cout << "  ...\n";
    for (Eigen::Index i = tail; i < dec.R_diag.size(); ++i) {
      std::cout << "  |R(" << i << "," << i << ")| = " << std::scientific
                << dec.R_diag(i);
      if (i >= dec.rank) std::cout << "  <-- 秩亏";
      std::cout << "\n";
    }
  }

  // 可辨识参数
  std::vector<std::string> full_names = regressor->getParameterNames(flags);
  std::cout << "\n可辨识参数 (" << dec.rank << "):\n";
  for (std::size_t i = 0; i < dec.identifiable_indices.size(); ++i) {
    Eigen::Index idx = dec.identifiable_indices[i];
    std::cout << "  [" << idx << "] "
              << full_names[static_cast<std::size_t>(idx)] << "\n";
  }

  std::cout << "\n不可辨识参数 (" << (dec.n_full - dec.rank) << "):\n";
  for (Eigen::Index i = 0; i < dec.n_full - dec.rank; ++i) {
    Eigen::Index idx = dec.unidentifiable_indices[static_cast<std::size_t>(i)];
    std::cout << "  [" << idx << "] "
              << full_names[static_cast<std::size_t>(idx)] << "\n";
  }

  // ---- 基参数组合公式 --------------------------------------------------------
  auto formulas = robot_dynamics::BaseParameterDecomposition::getBaseParameterFormulas(
      dec, full_names);
  auto C_matrix = robot_dynamics::BaseParameterDecomposition::getCombinationMatrix(dec);

  std::cout << "\n基参数组合公式 (β_base = C · β_full):\n";
  std::cout << "  代入 CAD 先验值: β_base_predicted = C · β_full_CAD\n";
  printSep();
  for (const auto &f : formulas) {
    std::cout << "  " << f << "\n";
  }
  printSep();

  // ---- 提取基回归矩阵并求解 ------------------------------------------------
  Eigen::MatrixXd W_base =
      robot_dynamics::BaseParameterDecomposition::getBaseRegressor(W_full, dec);

  std::cout << "\n基回归矩阵: " << W_base.rows() << " × " << W_base.cols() << "\n";

  auto solver = identification::createAlgorithm(algo_name, static_cast<int>(n_dof));
  if (!solver) {
    std::cerr << "错误: 不支持的算法 — " << algo_name << "\n";
    return 1;
  }
  solver->setUseRegularization(false); // 基回归矩阵列满秩，无需正则化

  Eigen::VectorXd beta_base = solver->solve(W_base, tau_stacked);

  // ---- 残差 (基空间) -------------------------------------------------------
  Eigen::VectorXd residual_base = W_base * beta_base - tau_stacked;
  double rmse_base = std::sqrt(residual_base.squaredNorm() /
                               static_cast<double>(tau_stacked.size()));
  double max_err_base = residual_base.cwiseAbs().maxCoeff();

  std::cout << "基参数解: 维度=" << beta_base.size()
            << ", RMSE=" << rmse_base << " Nm"
            << ", MaxErr=" << max_err_base << " Nm\n";

  // ---- 映射到全参数空间 ----------------------------------------------------
  Eigen::VectorXd beta_full =
      robot_dynamics::BaseParameterDecomposition::baseToFull(beta_base, dec);

  // ---- 残差 (全空间) -------------------------------------------------------
  Eigen::VectorXd residual_full = W_full * beta_full - tau_stacked;
  double rmse_full = std::sqrt(residual_full.squaredNorm() /
                               static_cast<double>(tau_stacked.size()));
  double max_err_full = residual_full.cwiseAbs().maxCoeff();

  std::cout << "全参数重建: 维度=" << beta_full.size()
            << ", RMSE=" << rmse_full << " Nm"
            << ", MaxErr=" << max_err_full << " Nm\n";

  // ---- 一致性检查 ----------------------------------------------------------
  double mismatch = (W_full * beta_full - W_base * beta_base).norm();
  std::cout << "一致性: ||W·β_full - W_base·β_base|| = " << std::scientific
            << mismatch << std::fixed << "\n";

  // C 矩阵一致性: C · β_full 应等于 β_base (机器精度内)
  Eigen::VectorXd beta_base_check = C_matrix * beta_full;
  double c_mismatch = (beta_base_check - beta_base).norm();
  std::cout << "C 矩阵验证: ||C·β_full - β_base|| = " << std::scientific
            << c_mismatch << std::fixed << "\n";

  // ---- 输出全参数 ----------------------------------------------------------
  printSep();
  std::cout << "全参数重建 (惯性参数, 每 body 一行 10 个):\n";
  for (std::size_t b = 0; b < n_bodies; ++b) {
    Eigen::Index base = static_cast<Eigen::Index>(b * 10);
    std::cout << "  body " << b << ": [";
    for (int j = 0; j < 10; ++j) {
      std::cout << std::scientific << std::setprecision(4) << beta_full(base + j);
      if (j < 9) std::cout << ", ";
    }
    std::cout << "]\n";
  }
  if (armature) {
    Eigen::Index arm_start = static_cast<Eigen::Index>(n_bodies * 10);
    std::cout << "  armature: [";
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(n_dof); ++j) {
      std::cout << beta_full(arm_start + j);
      if (j + 1 < static_cast<Eigen::Index>(n_dof)) std::cout << ", ";
    }
    std::cout << "]\n";
  }

  // ---- 保存 YAML ----------------------------------------------------------
  saveBaseResults(output_file, robot_name, algo_name,
                  (friction_loaded ? friction_yaml : "none"),
                  friction_loaded, armature, dec,
                  beta_base, beta_full,
                  rmse_base, max_err_base,
                  rmse_full, max_err_full,
                  n_bodies, full_names,
                  formulas, C_matrix);

  return 0;
}
