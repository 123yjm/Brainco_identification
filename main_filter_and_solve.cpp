/**
 * @file main_filter_and_solve.cpp
 * @brief 滤波 + 辨识一站式入口（无中间 CSV 落盘）
 *
 * 读取 butterworth_filter.yaml → 原始 .txt → 内存滤波 →
 * ExperimentData → identification.yaml 配置的辨识算法 → 结果 .yaml
 *
 * 用法:
 *   ./filter_and_solve [选项]
 *
 * 选项:
 *   --filter-config <yaml>  滤波配置文件 (默认: config/butterworth_filter.yaml)
 *   --solve-config <yaml>   辨识配置文件 (默认: config/identification.yaml)
 *   --passband <Hz>         覆盖通带频率
 *   --stopband <Hz>         覆盖阻带频率
 *   --algo <name>           覆盖辨识算法 (OLS/WLS/IRLS/TLS/EKF/NLS_FRICTION)
 *   --output <yaml>         覆盖结果输出路径
 *   --no-damping            禁用阻尼项辨识
 *   --help                  打印帮助信息
 */

#include "algorithms.hpp"
#include "butterworth_filter.hpp"
#include "csv_io.hpp"
#include "data_loader.hpp"
#include "regressor_factory.hpp"

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
#include <unordered_map>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

using signal_processing::FilterConfig;

namespace {

// =========================================================================
// 算法名称映射（来自 main_solve.cpp）
// =========================================================================
const std::unordered_map<int, std::string> ALGO_MAP = {
    {1, "OLS"}, {2, "WLS"}, {3, "IRLS"},
    {4, "TLS"}, {5, "EKF"}, {8, "NLS_FRICTION"},
};

const std::vector<std::string> BENCHMARK_ALGOS = {
    "OLS", "WLS", "IRLS", "TLS", "EKF", "NLS_FRICTION",
};

// =========================================================================
// 路径解析
// =========================================================================
inline std::string resolvePath(const std::string& path) {
    std::filesystem::path p(path);
    if (p.is_absolute()) return path;
    return (std::filesystem::path(PROJECT_ROOT_DIR) / p).string();
}

// =========================================================================
// 简易 YAML 解析（key: value，来自 main_solve.cpp）
// =========================================================================
std::unordered_map<std::string, std::string> parseSimpleYaml(
    const std::string& filepath) {
    std::unordered_map<std::string, std::string> kv;
    std::ifstream f(filepath);
    if (!f) {
        std::cerr << "警告: 无法打开配置文件 " << filepath
                  << "，使用默认值。" << std::endl;
        return kv;
    }
    std::string line;
    while (std::getline(f, line)) {
        auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        if (line[first] == '#') continue;
        auto last = line.find_last_not_of(" \t\r\n");
        line = line.substr(first, last - first + 1);

        auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = line.substr(0, colon);
        std::string val = line.substr(colon + 1);
        auto klast = key.find_last_not_of(" \t");
        if (klast != std::string::npos) key = key.substr(0, klast + 1);
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

// =========================================================================
// 辨识配置
// =========================================================================
struct SolveOptions {
    std::string solver_config = "config/identification.yaml";
    std::string robot;
    std::string kinematic_params;
    std::string output_file = "result/identification.yaml";
    int algorithm = 0;          // 0=基准全部
    std::string algo_name;      // 命令行 --algo 覆盖
    bool regularization = true;
    bool damping = true;
};

SolveOptions loadSolveConfig(int argc, char* argv[]) {
    SolveOptions opt;

    // 第一遍: 解析 --solve-config
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--solve-config" && i + 1 < argc)
            opt.solver_config = argv[++i];
    }
    opt.solver_config = resolvePath(opt.solver_config);

    // 读取 identification.yaml
    auto cfg = parseSimpleYaml(opt.solver_config);
    if (cfg.count("robot"))               opt.robot             = cfg["robot"];
    if (cfg.count("kinematic_params"))     opt.kinematic_params  = cfg["kinematic_params"];
    if (cfg.count("output_file"))         opt.output_file       = cfg["output_file"];
    if (cfg.count("algorithm"))           opt.algorithm         = std::stoi(cfg["algorithm"]);
    if (cfg.count("regularization"))      opt.regularization    = (std::stoi(cfg["regularization"]) != 0);

    opt.kinematic_params = resolvePath(opt.kinematic_params);
    opt.output_file      = resolvePath(opt.output_file);

    // 命令行覆盖
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--solve-config" && i + 1 < argc) { ++i; continue; }
        if (arg == "--filter-config" && i + 1 < argc) { ++i; continue; }
        if (arg == "--passband" && i + 1 < argc)     { ++i; continue; }
        if (arg == "--stopband" && i + 1 < argc)     { ++i; continue; }
        if (arg == "--algo" && i + 1 < argc)
            opt.algo_name = argv[++i];
        else if (arg == "--output" && i + 1 < argc)
            opt.output_file = resolvePath(argv[++i]);
        else if (arg == "--no-damping")
            opt.damping = false;
    }

    // 路径解析
    if (!opt.kinematic_params.empty())
        opt.kinematic_params = resolvePath(opt.kinematic_params);
    opt.output_file = resolvePath(opt.output_file);

    return opt;
}

// =========================================================================
// 桥接: FilteredOutputData → ExperimentData
// =========================================================================
ExperimentData filteredToExperiment(
    const signal_processing::FilteredOutputData& src) {
    ExperimentData dst;
    dst.time      = src.time;
    dst.q         = src.q_filtered;
    dst.qd        = src.q_dot_filtered;
    dst.qdd       = src.q_ddot_filtered;
    dst.tau       = src.tau_filtered;
    dst.n_samples = static_cast<std::size_t>(src.n_samples);
    dst.n_dof     = static_cast<std::size_t>(src.n_dof);
    return dst;
}

// =========================================================================
// 保存辨识结果（来自 main_solve.cpp）
// =========================================================================
void saveResults(const std::string& path, const std::string& algo_name,
                 const Eigen::VectorXd& params, std::size_t /*num_params*/,
                 double rmse, double max_error,
                 const std::string& robot_name,
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

// =========================================================================
// 单次辨识（来自 main_solve.cpp）
// =========================================================================
struct SingleResult {
    std::string algo;
    Eigen::VectorXd beta;
    double rmse;
    double max_err;
};

SingleResult runSingle(const ExperimentData& data,
                       const robot_dynamics::IDynamicsRegressor& regressor,
                       robot_dynamics::ParamFlags flags,
                       const std::string& algo_name, bool use_reg) {
    const std::size_t num_params = regressor.numParameters(flags);
    const std::size_t dof = regressor.nDof();
    const Eigen::Index K = static_cast<Eigen::Index>(data.n_samples);
    const Eigen::Index rows_per = static_cast<Eigen::Index>(dof);

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

    auto solver = identification::createAlgorithm(algo_name, dof);
    if (!solver) {
        std::cerr << "跳过不支持的算法: " << algo_name << std::endl;
        return {algo_name, Eigen::VectorXd::Zero(num_params), 0.0, 0.0};
    }
    solver->setUseRegularization(use_reg);

    if (auto* nls = dynamic_cast<identification::NonlinearFrictionLM*>(solver.get()))
        nls->setVelocityData(data.qd);

    Eigen::VectorXd beta = solver->solve(W_total, tau_stacked);

    Eigen::VectorXd residual = W_total * beta - tau_stacked;
    double rmse = std::sqrt(residual.squaredNorm() / static_cast<double>(tau_stacked.size()));
    double max_err = residual.cwiseAbs().maxCoeff();

    return {algo_name, beta, rmse, max_err};
}

// =========================================================================
// 打印帮助
// =========================================================================
void printHelp(const char* prog) {
    std::cout << "滤波 + 辨识一站式管线\n"
              << "用法: " << prog << " [选项]\n\n"
              << "选项:\n"
              << "  --filter-config <yaml>  滤波配置文件 (默认: config/butterworth_filter.yaml)\n"
              << "  --solve-config <yaml>   辨识配置文件 (默认: config/identification.yaml)\n"
              << "  --passband <Hz>         覆盖通带频率\n"
              << "  --stopband <Hz>         覆盖阻带频率\n"
              << "  --algo <name>           覆盖辨识算法 (OLS/WLS/IRLS/TLS/EKF/NLS_FRICTION)\n"
              << "  --output <yaml>         覆盖结果输出路径\n"
              << "  --no-damping            禁用阻尼项辨识\n"
              << "  --help                  打印帮助信息\n"
              << std::endl;
}

}  // anonymous namespace

// ============================================================================
int main(int argc, char* argv[]) {
    // ---- 解析 --help -------------------------------------------------------
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--help") {
            printHelp(argv[0]);
            return 0;
        }
    }

    // ---- 1. 加载滤波配置 ---------------------------------------------------
    std::string filter_config_path = "config/butterworth_filter.yaml";
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--filter-config" && i + 1 < argc)
            filter_config_path = argv[++i];
    }
    filter_config_path = resolvePath(filter_config_path);
    FilterConfig filter_cfg = signal_processing::loadFilterConfig(
        filter_config_path, PROJECT_ROOT_DIR);

    // CLI 覆盖滤波参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--passband" && i + 1 < argc)
            filter_cfg.passband_hz = std::stod(argv[++i]);
        else if (arg == "--stopband" && i + 1 < argc)
            filter_cfg.stopband_hz = std::stod(argv[++i]);
    }

    // ---- 2. 加载辨识配置 ---------------------------------------------------
    SolveOptions solve_opt = loadSolveConfig(argc, argv);

    // ---- 3. 打印配置 -------------------------------------------------------
    std::cout << "═══════════════════════════════════════════\n"
              << "  滤波 + 辨识 一站式管线\n"
              << "═══════════════════════════════════════════\n"
              << "滤波配置:     " << filter_config_path << "\n"
              << "辨识配置:     " << solve_opt.solver_config << "\n"
              << "输入文件:     " << filter_cfg.input_txt << "\n"
              << "通带频率:     " << filter_cfg.passband_hz << " Hz\n"
              << "阻带频率:     " << filter_cfg.stopband_hz << " Hz\n"
              << "机器人:       " << solve_opt.robot << "\n"
              << "输出:         " << solve_opt.output_file << "\n"
              << std::endl;

    // ---- 4. 读取原始数据 ---------------------------------------------------
    std::cout << "读取原始数据..." << std::endl;
    signal_processing::RawMeasurementData raw;
    try {
        raw = signal_processing::readRawTxt(filter_cfg.input_txt);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "  样本数: " << raw.n_samples
              << ", DOF: " << raw.n_dof << std::endl;

    // ---- 5. 设计滤波器 -----------------------------------------------------
    double fs = filter_cfg.fs;
    double Wp = filter_cfg.passband_hz / (fs / 2.0);
    double Ws = filter_cfg.stopband_hz / (fs / 2.0);

    std::cout << "设计 Butterworth 低通滤波器...\n"
              << "  归一化通带: " << Wp
              << ", 归一化阻带: " << Ws << std::endl;

    signal_processing::ButterworthFilterDesign design;
    try {
        design = signal_processing::designButterworthLowpass(
            Wp, Ws, filter_cfg.rp_db, filter_cfg.rs_db, fs);
    } catch (const std::exception& e) {
        std::cerr << "滤波器设计失败: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "  阶数: " << design.order
              << ", 归一化截止: " << design.cutoff_normalized << std::endl;

    // ---- 6. 内存滤波（不写 CSV）--------------------------------------------
    double Ts = 1.0 / fs;
    std::cout << "滤波中..." << std::endl;

    signal_processing::FilteredOutputData filtered;
    filtered.n_samples = raw.n_samples;
    filtered.n_dof     = raw.n_dof;

    filtered.time.resize(raw.n_samples);
    double t0 = raw.t_abs[0];
    for (int i = 0; i < raw.n_samples; ++i)
        filtered.time[i] = raw.t_abs[i] - t0;

    filtered.q_filtered      = raw.q;                                    // 不滤波
    filtered.q_dot_filtered  = signal_processing::filtfilt(design.b, design.a, raw.q_dot);
    filtered.q_ddot_filtered = signal_processing::centralDifference(filtered.q_dot_filtered, Ts);
    filtered.tau_filtered    = signal_processing::filtfilt(design.b, design.a, raw.motor_current);
    std::cout << "  滤波完成" << std::endl;

    // ---- 7. 桥接 → ExperimentData ------------------------------------------
    ExperimentData data = filteredToExperiment(filtered);

    // ---- 8. 加载机器人模型 -------------------------------------------------
    std::cout << "加载机器人模型: " << solve_opt.robot
              << " (" << solve_opt.kinematic_params << ")" << std::endl;
    std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor;
    try {
        regressor = robot_dynamics::RegressorFactory::create(
            solve_opt.robot, solve_opt.kinematic_params);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    // ---- 9. flags ----------------------------------------------------------
    auto flags = robot_dynamics::ParamFlags::NONE;
    if (solve_opt.damping)
        flags = flags | robot_dynamics::ParamFlags::DAMPING;

    const std::size_t num_params = regressor->numParameters(flags);
    std::cout << "参数数: " << num_params
              << "  (damping=" << (solve_opt.damping ? "yes" : "no")
              << ", reg=" << (solve_opt.regularization ? "yes" : "no") << ")"
              << std::endl;

    // ---- 10. 确定算法列表 --------------------------------------------------
    std::vector<std::string> algos_to_run;
    if (!solve_opt.algo_name.empty()) {
        algos_to_run = {solve_opt.algo_name};
    } else if (solve_opt.algorithm == 0) {
        algos_to_run = BENCHMARK_ALGOS;
    } else {
        auto it = ALGO_MAP.find(solve_opt.algorithm);
        if (it != ALGO_MAP.end())
            algos_to_run = {it->second};
        else {
            std::cerr << "无效的 algorithm 编号: " << solve_opt.algorithm << std::endl;
            return 1;
        }
    }

    std::cout << "运行算法: ";
    for (auto& a : algos_to_run) std::cout << a << " ";
    std::cout << std::endl;

    // ---- 11. 运行辨识 ------------------------------------------------------
    std::vector<SingleResult> results;
    bool append = false;
    for (const auto& algo : algos_to_run) {
        std::cout << "\n--- " << algo << " ---" << std::endl;
        auto r = runSingle(data, *regressor, flags, algo, solve_opt.regularization);
        std::cout << algo << " => RMSE: " << r.rmse
                  << " Nm, Max Error: " << r.max_err << " Nm" << std::endl;
        saveResults(solve_opt.output_file, algo, r.beta, num_params,
                    r.rmse, r.max_err, solve_opt.robot, append);
        append = true;
        if (r.beta.size() > 0) results.push_back(r);
    }

    // ---- 12. 总结 ----------------------------------------------------------
    if (results.size() > 1) {
        std::cout << "\n===== 基准测试总结 =====" << std::endl;
        std::cout << std::left << std::setw(18) << "Algorithm"
                  << std::setw(16) << "RMSE (Nm)"
                  << "Max Error (Nm)" << std::endl;
        for (const auto& r : results) {
            std::cout << std::left << std::setw(18) << r.algo
                      << std::setw(16) << r.rmse
                      << r.max_err << std::endl;
        }
    }

    std::cout << "\n结果已保存到: " << solve_opt.output_file << std::endl;
    return 0;
}
