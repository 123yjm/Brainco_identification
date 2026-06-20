/**
 * @file main.cpp
 * @brief brainco_identification — revoarm_new 线性回归辨识入口
 *
 * 读取 config/identification.yaml 获取配置，支持命令行参数覆盖。
 * 机器人运动学参数从 kinematic_params.yaml 加载。
 * algorithm: 0=基准全部, 1=OLS, 2=WLS, 3=IRLS, 4=TLS, 5=EKF, 6=ML, 8=NLS_FRICTION
 *
 * 用法:
 *   ./identify [--config <yaml>] [--data <csv>] [--algo <name>] [--output <yaml>]
 *              [--no-damping]
 */

#include "data_loader.hpp"
#include "algorithms.hpp"
#include "regressor_factory.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>

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
#include <unordered_map>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 算法编号 → 名称映射
// ---------------------------------------------------------------------------
const std::unordered_map<int, std::string> ALGO_MAP = {
    {1, "OLS"},          {2, "WLS"},           {3, "IRLS"},
    {4, "TLS"},          {5, "EKF"},           {6, "ML"},
    {7, "CLOE"},         {8, "NLS_FRICTION"},
};

// 基准测试所用的算法列表 (algorithm=0 时按此顺序跑全部)
const std::vector<std::string> BENCHMARK_ALGOS = {
    "OLS", "WLS", "IRLS", "TLS", "EKF", "ML", "CLOE", "NLS_FRICTION",
};

// ---------------------------------------------------------------------------
// 项目根目录
// ---------------------------------------------------------------------------
#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

inline std::string resolvePath(const std::string &path) {
  std::filesystem::path p(path);
  if (p.is_absolute()) return path;
  return (std::filesystem::path(PROJECT_ROOT_DIR) / p).string();
}

// ---------------------------------------------------------------------------
// 简易 YAML 配置解析 (仅处理 key: value 格式，忽略注释和空行)
// ---------------------------------------------------------------------------
std::unordered_map<std::string, std::string> parseSimpleYaml(const std::string &filepath) {
  std::unordered_map<std::string, std::string> kv;
  std::ifstream f(filepath);
  if (!f) {
    std::cerr << "警告: 无法打开配置文件 " << filepath
              << "，使用默认值。" << std::endl;
    return kv;
  }
  std::string line;
  while (std::getline(f, line)) {
    // 去掉行首尾空白
    auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) continue;
    if (line[first] == '#') continue; // 注释行
    auto last = line.find_last_not_of(" \t\r\n");
    line = line.substr(first, last - first + 1);

    auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = line.substr(0, colon);
    std::string val = line.substr(colon + 1);
    // 去掉 key 尾空白
    auto klast = key.find_last_not_of(" \t");
    if (klast != std::string::npos) key = key.substr(0, klast + 1);
    // 去掉 val 首尾空白和引号
    auto vfirst = val.find_first_not_of(" \t");
    if (vfirst == std::string::npos) continue;
    val = val.substr(vfirst);
    auto vlast = val.find_last_not_of(" \t\r\n");
    if (vlast != std::string::npos) val = val.substr(0, vlast + 1);
    if (val.size() >= 2 && val.front() == '"' && val.back() == '"')
      val = val.substr(1, val.size() - 2);

    kv[key] = val;
  }
  return kv;
}

// ---------------------------------------------------------------------------
// 选项结构
// ---------------------------------------------------------------------------
struct Options {
  std::string config_file = "config/identification.yaml";
  std::string data_file;
  std::string output_file;
  std::string kinematic_params;
  std::string robot;        // identification.yaml 中的 robot 字段
  int algorithm = 0;       // 0=基准全部, 1-8 对应具体算法
  std::string algo_name;   // 命令行 --algo 覆盖（优先级最高）
  bool regularization = true;
  bool damping = true;
};

Options loadConfig(int argc, char *argv[]) {
  Options opt;

  // ---- 1. 解析命令行参数（先收集 --config 路径）---------------------------
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      opt.config_file = argv[++i];
    }
  }
  opt.config_file = resolvePath(opt.config_file);

  // ---- 2. 读取配置文件 ----------------------------------------------------
  auto cfg = parseSimpleYaml(opt.config_file);
  if (cfg.count("data_file"))       opt.data_file       = cfg["data_file"];
  if (cfg.count("output_file"))     opt.output_file     = cfg["output_file"];
  if (cfg.count("algorithm"))       opt.algorithm       = std::stoi(cfg["algorithm"]);
  if (cfg.count("regularization"))  opt.regularization  = (std::stoi(cfg["regularization"]) != 0);
  if (cfg.count("kinematic_params")) opt.kinematic_params = cfg["kinematic_params"];
  if (cfg.count("robot"))             opt.robot            = cfg["robot"];

  // 默认值
  if (opt.output_file.empty())
    opt.output_file = "result/identification.yaml";

  // ---- 3. 命令行参数覆盖 --------------------------------------------------
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) { ++i; continue; }
    if (arg == "--data" && i + 1 < argc)
      opt.data_file = argv[++i];
    else if (arg == "--algo" && i + 1 < argc)
      opt.algo_name = argv[++i];
    else if (arg == "--output" && i + 1 < argc)
      opt.output_file = argv[++i];
    else if (arg == "--kinematic-params" && i + 1 < argc)
      opt.kinematic_params = argv[++i];
    else if (arg == "--no-damping")
      opt.damping = false;
  }

  // 路径解析为绝对路径（跳过空字符串）
  if (!opt.data_file.empty())
    opt.data_file         = resolvePath(opt.data_file);
  opt.output_file       = resolvePath(opt.output_file);
  if (!opt.kinematic_params.empty())
    opt.kinematic_params = resolvePath(opt.kinematic_params);

  return opt;
}

// ---------------------------------------------------------------------------
// 保存结果为 YAML
// ---------------------------------------------------------------------------
void saveResults(const std::string &path, const std::string &algo_name,
                 const Eigen::VectorXd &params, std::size_t num_params,
                 double rmse, double max_error,
                 const std::string &robot_name,
                 bool append_mode = false) {
  std::filesystem::path p(path);
  if (p.has_parent_path())
    std::filesystem::create_directories(p.parent_path());

  auto mode = append_mode ? std::ios::app : std::ios::trunc;
  std::ofstream out(path, mode);
  if (!out) {
    std::cerr << "无法写入结果文件: " << path << std::endl;
    return;
  }
  std::time_t now = std::time(nullptr);
  char time_buf[64];
  std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S",
                std::localtime(&now));

  out << std::scientific << std::setprecision(15);

  if (!append_mode) {
    out << "calibration_date: \"" << time_buf << "\"\n";
    out << "evaluation_method: \"torque_residual_rmse\"\n";
    out << "mode: \"BENCHMARK_ALL\"\n";
    out << "robot: \"" << robot_name << "\"\n";
    out << "benchmark_results:\n";
  }

  out << "  - algorithm: \"" << algo_name << "\"\n";
  out << "    torque_rmse: " << rmse << "\n";
  out << "    torque_max_error: " << max_error << "\n";
  out << "    parameters:\n";
  for (Eigen::Index i = 0; i < params.size(); ++i)
    out << "      - " << params(i) << "\n";
}

// ---------------------------------------------------------------------------
// 单次辨识
// ---------------------------------------------------------------------------
struct SingleResult {
  std::string algo;
  Eigen::VectorXd beta;
  double rmse;
  double max_err;
};

SingleResult runSingle(const ExperimentData &data,
                       const robot_dynamics::IDynamicsRegressor &regressor,
                       robot_dynamics::ParamFlags flags,
                       const std::string &algo_name, bool use_reg) {
  const std::size_t num_params = regressor.numParameters(flags);
  const std::size_t dof = regressor.nDof();
  const Eigen::Index K = static_cast<Eigen::Index>(data.n_samples);
  const Eigen::Index rows_per = static_cast<Eigen::Index>(dof);

  // 构建 W 和 tau_stacked
  Eigen::MatrixXd W_total =
      Eigen::MatrixXd::Zero(rows_per * K, static_cast<Eigen::Index>(num_params));
  Eigen::VectorXd tau_stacked(rows_per * K);

  for (Eigen::Index k = 0; k < K; ++k) {
    Eigen::VectorXd q   = data.q.row(k).transpose();
    Eigen::VectorXd qd  = data.qd.row(k).transpose();
    Eigen::VectorXd qdd = data.qdd.row(k).transpose();
    Eigen::MatrixXd Y_k = regressor.computeRegressorMatrix(q, qd, qdd, flags);
    W_total.block(k * rows_per, 0, rows_per, static_cast<Eigen::Index>(num_params)) = Y_k;
    for (Eigen::Index j = 0; j < rows_per; ++j)
      tau_stacked(k * rows_per + j) = data.tau(k, j);
  }

  // 求解
  auto solver = identification::createAlgorithm(algo_name, dof);
  if (!solver) {
    std::cerr << "跳过不支持的算法: " << algo_name << std::endl;
    return {algo_name, Eigen::VectorXd::Zero(num_params), 0.0, 0.0};
  }
  solver->setUseRegularization(use_reg);

  if (auto *nls = dynamic_cast<identification::NonlinearFrictionLM *>(solver.get()))
    nls->setVelocityData(data.qd);

  Eigen::VectorXd beta = solver->solve(W_total, tau_stacked);

  // 评估
  Eigen::VectorXd residual = W_total * beta - tau_stacked;
  double rmse = std::sqrt(residual.squaredNorm() / static_cast<double>(tau_stacked.size()));
  double max_err = residual.cwiseAbs().maxCoeff();

  return {algo_name, beta, rmse, max_err};
}

} // namespace

// ============================================================================
int main(int argc, char *argv[]) {
  Options opt = loadConfig(argc, argv);

  // ---- 校验必填字段 -------------------------------------------------------
  if (opt.robot.empty()) {
    std::cerr << "错误: 缺少必填配置项 robot (identification.yaml 中的 robot 字段)"
              << std::endl;
    return 1;
  }
  if (opt.kinematic_params.empty()) {
    std::cerr << "错误: 缺少必填配置项 kinematic_params (机器人运动学参数 YAML 路径)"
              << std::endl;
    return 1;
  }
  if (opt.data_file.empty()) {
    std::cerr << "错误: 缺少必填配置项 data_file (数据 CSV 路径)" << std::endl;
    return 1;
  }

  // ---- 加载机器人模型 -----------------------------------------------------
  std::cout << "加载机器人模型: " << opt.robot
            << " (" << opt.kinematic_params << ")" << std::endl;
  std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor;
  try {
    regressor = robot_dynamics::RegressorFactory::create(opt.robot,
                                                          opt.kinematic_params);
  } catch (const std::exception &e) {
    std::cerr << "错误: " << e.what() << std::endl;
    return 1;
  }
  const std::size_t DOF = regressor->nDof();

  // ---- 加载数据 -----------------------------------------------------------
  std::cout << "加载数据: " << opt.data_file << std::endl;
  ExperimentData data;
  try {
    data = DataLoader::loadCSV(opt.data_file, DOF);
  } catch (const std::exception &e) {
    std::cerr << "加载失败: " << e.what() << std::endl;
    return 1;
  }
  std::cout << "样本数: " << data.n_samples << std::endl;

  // ---- flags --------------------------------------------------------------
  auto flags = robot_dynamics::ParamFlags::NONE;
  if (opt.damping)  flags = flags | robot_dynamics::ParamFlags::DAMPING;

  const std::size_t num_params = regressor->numParameters(flags);
  std::cout << "参数数: " << num_params
            << "  (damping=" << (opt.damping ? "yes" : "no")
            << ", regularization=" << (opt.regularization ? "yes" : "no") << ")"
            << std::endl;

  // ---- 确定待运行的算法列表 ------------------------------------------------
  std::vector<std::string> algos_to_run;
  if (!opt.algo_name.empty()) {
    // 命令行 --algo 优先级最高
    algos_to_run = {opt.algo_name};
  } else if (opt.algorithm == 0) {
    // 基准全部
    algos_to_run = BENCHMARK_ALGOS;
  } else {
    auto it = ALGO_MAP.find(opt.algorithm);
    if (it != ALGO_MAP.end())
      algos_to_run = {it->second};
    else {
      std::cerr << "无效的 algorithm 编号: " << opt.algorithm << std::endl;
      return 1;
    }
  }

  std::cout << "运行算法: ";
  for (auto &a : algos_to_run) std::cout << a << " ";
  std::cout << std::endl;

  // ---- 运行辨识 -----------------------------------------------------------
  std::vector<SingleResult> results;
  bool append = false;
  for (const auto &algo : algos_to_run) {
    std::cout << "\n--- " << algo << " ---" << std::endl;
    auto r = runSingle(data, *regressor, flags, algo, opt.regularization);
    std::cout << algo << " => RMSE: " << r.rmse << " Nm, Max Error: "
              << r.max_err << " Nm" << std::endl;
    saveResults(opt.output_file, algo, r.beta, num_params, r.rmse, r.max_err,
                opt.robot, append);
    append = true; // 后续算法追加写入
    if (r.beta.size() > 0) results.push_back(r);
  }

  // ---- 总结 ---------------------------------------------------------------
  if (results.size() > 1) {
    std::cout << "\n===== 基准测试总结 =====" << std::endl;
    std::cout << std::left << std::setw(18) << "Algorithm"
              << std::setw(16) << "RMSE (Nm)"
              << "Max Error (Nm)" << std::endl;
    for (const auto &r : results) {
      std::cout << std::left << std::setw(18) << r.algo
                << std::setw(16) << r.rmse
                << r.max_err << std::endl;
    }
  }

  std::cout << "\n结果已保存到: " << opt.output_file << std::endl;
  return 0;
}
