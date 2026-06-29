/**
 * @file main_filter_and_solve.cpp
 * @brief 滤波 + 辨识一站式入口（无中间 CSV 落盘）
 *
 * 用法: ./filter_and_solve --robot <robot_dir> [--algo <name>] [--passband <Hz>]
 *                          [--stopband <Hz>] [--no-damping] [--help]
 */

#include "algorithms.hpp"
#include "butterworth_filter.hpp"
#include "csv_io.hpp"
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
#include <string>
#include <unordered_map>
#include <vector>

namespace {

const std::unordered_map<int, std::string> ALGO_MAP = {
    {1, "OLS"}, {2, "WLS"}, {3, "IRLS"},
    {4, "TLS"}, {5, "EKF"}, {8, "NLS_FRICTION"},
};

const std::vector<std::string> BENCHMARK_ALGOS = {
    "OLS", "WLS", "IRLS", "TLS", "EKF", "NLS_FRICTION",
};

struct SingleResult {
    std::string algo;
    Eigen::VectorXd beta;
    double rmse;
    double max_err;
};

ExperimentData filteredToExperiment(const signal_processing::FilteredOutputData& src) {
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

SingleResult runSingle(const ExperimentData& data,
                       const robot_dynamics::IDynamicsRegressor& regressor,
                       robot_dynamics::ParamFlags flags,
                       const std::string& algo_name, bool use_reg) {
    const std::size_t num_params = regressor.numParameters(flags);
    const std::size_t dof = regressor.nDof();
    const Eigen::Index K = static_cast<Eigen::Index>(data.n_samples);
    const Eigen::Index rows_per = static_cast<Eigen::Index>(dof);

    Eigen::MatrixXd W_total = Eigen::MatrixXd::Zero(rows_per * K, static_cast<Eigen::Index>(num_params));
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
    if (!solver) return {algo_name, Eigen::VectorXd::Zero(num_params), 0.0, 0.0};
    solver->setUseRegularization(use_reg);
    if (auto* nls = dynamic_cast<identification::NonlinearFrictionLM*>(solver.get()))
        nls->setVelocityData(data.qd);
    Eigen::VectorXd beta = solver->solve(W_total, tau_stacked);
    Eigen::VectorXd residual = W_total * beta - tau_stacked;
    double rmse = std::sqrt(residual.squaredNorm() / static_cast<double>(tau_stacked.size()));
    double max_err = residual.cwiseAbs().maxCoeff();
    return {algo_name, beta, rmse, max_err};
}

void saveResults(const std::string& path, const std::string& algo_name,
                 const Eigen::VectorXd& params,
                 double rmse, double max_error,
                 const std::string& robot_name, bool append_mode,
                 std::size_t n_bodies, std::size_t n_dof) {
    std::filesystem::path p(path);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    auto mode = append_mode ? std::ios::app : std::ios::trunc;
    std::ofstream out(path, mode);
    if (!out) { std::cerr << "无法写入: " << path << std::endl; return; }
    std::time_t now = std::time(nullptr);
    char buf[64]; std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    out << std::scientific << std::setprecision(15);
    if (!append_mode) {
        out << "calibration_date: \"" << buf << "\"\n"
            << "robot: \"" << robot_name << "\"\n"
            << "evaluation_method: \"torque_residual_rmse\"\n"
            << "benchmark_results:\n";
    }
    out << "  - algorithm: \"" << algo_name << "\"\n"
        << "    torque_rmse: " << rmse << "\n"
        << "    torque_max_error: " << max_error << "\n"
        << "    parameters:\n"
        << "        [\n";

    Eigen::Index inertial_count = static_cast<Eigen::Index>(n_bodies * 10);
    Eigen::Index total = params.size();

    // 惯性组（每行 10 个）
    for (Eigen::Index i = 0; i < inertial_count; i += 10) {
        out << "        ";
        for (Eigen::Index j = 0; j < 10 && i + j < inertial_count; ++j) {
            out << params(i + j);
            if (i + j + 1 < total) out << ",";
        }
        out << "\n";
    }

    // damping 组
    if (total > inertial_count) {
        out << "\n        ";
        for (Eigen::Index j = inertial_count; j < total; ++j) {
            out << params(j);
            if (j + 1 < total) out << ",";
        }
        out << "\n";
    }

    out << "        ]\n";
}

void printHelp(const char* prog) {
    std::cout << "滤波 + 辨识一站式管线\n"
              << "用法: " << prog << " --robot <robot_dir> [选项]\n\n"
              << "选项:\n"
              << "  --robot <dir>       机器人目录 (如 robots/revoarm_right)\n"
              << "  --passband <Hz>     覆盖通带频率\n"
              << "  --stopband <Hz>     覆盖阻带频率\n"
              << "  --algo <name>       算法 (OLS/WLS/IRLS/TLS/EKF/NLS_FRICTION)\n"
              << "  --no-damping        禁用阻尼项辨识\n"
              << "  --help              打印帮助信息\n";
}

// ---- PC (Physical Consistent) 辨识辅助函数 ---------------------------------

/// 从 inertia_prior.yaml 加载惯性先验参数，拼接为 (n_bodies*10) 维向量

}  // anonymous namespace

int main(int argc, char* argv[]) {
    std::string robot_dir;
    std::string algo_name;
    bool damping = true;
    int algorithm = 3;
    bool regularization = true;
    double passband_override = -1, stopband_override = -1;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") { printHelp(argv[0]); return 0; }
        else if ((arg == "--robot" || arg == "-r") && i + 1 < argc) robot_dir = robot_utils::resolvePath(robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
        else if (arg == "--algo" && i + 1 < argc) algo_name = argv[++i];
        else if (arg == "--passband" && i + 1 < argc) passband_override = std::stod(argv[++i]);
        else if (arg == "--stopband" && i + 1 < argc) stopband_override = std::stod(argv[++i]);
        else if (arg == "--no-damping") damping = false;
    }

    if (robot_dir.empty()) {
        std::cerr << "错误: 需要 --robot <dir>\n";
        return 1;
    }

    std::string robot_name = robot_utils::robotNameFromDir(robot_dir);

    // ---- 1. 滤波 ----------------------------------------------------------
    auto filter_cfg = signal_processing::loadFilterConfig(
        robot_utils::configPath(robot_dir, "butterworth_filter.yaml"));
    if (passband_override > 0) filter_cfg.passband_hz = passband_override;
    if (stopband_override > 0) filter_cfg.stopband_hz = stopband_override;

    std::string input_file = robot_utils::findFirstFile(
        robot_utils::dataInertiaPath(robot_dir, ""), "*.csv");
    if (input_file.empty()) {
        input_file = robot_utils::findFirstFile(
            robot_utils::dataInertiaPath(robot_dir, ""), "*.txt");
    }
    if (input_file.empty()) {
        std::cerr << "错误: 未找到 .csv 或 .txt 文件\n"; return 1;
    }

    std::cout << "═══════════════════════════════════════════\n"
              << "  滤波 + 辨识 一站式管线\n"
              << "═══════════════════════════════════════════\n"
              << "机器人: " << robot_name << "\n"
              << "输入:   " << input_file << "\n"
              << "通带:   " << filter_cfg.passband_hz << " Hz\n";

    auto raw = signal_processing::readRawTxt(input_file);
    double fs = filter_cfg.fs;
    double Wp = filter_cfg.passband_hz / (fs / 2.0);
    double Ws = filter_cfg.stopband_hz / (fs / 2.0);
    auto design = signal_processing::designButterworthLowpass(
        Wp, Ws, filter_cfg.rp_db, filter_cfg.rs_db, fs);

    signal_processing::FilteredOutputData filtered;
    filtered.n_samples = raw.n_samples;
    filtered.n_dof     = raw.n_dof;
    filtered.time.resize(raw.n_samples);
    double t0 = raw.t_abs[0];
    for (int i = 0; i < raw.n_samples; ++i) filtered.time[i] = raw.t_abs[i] - t0;
    filtered.q_filtered = raw.q;

    if (filter_cfg.turn_on_filter) {
        filtered.q_dot_filtered  = signal_processing::filtfilt(design.b, design.a, raw.q_dot);
        filtered.tau_filtered    = signal_processing::filtfilt(design.b, design.a, raw.motor_current);
    } else {
        filtered.q_dot_filtered  = raw.q_dot;
        filtered.tau_filtered    = raw.motor_current;
    }
    filtered.q_ddot_filtered = signal_processing::centralDifference(filtered.q_dot_filtered, 1.0/fs);
    std::cout << "滤波完成, 样本: " << filtered.n_samples << std::endl;

    // ---- 2. 辨识 ----------------------------------------------------------
    ExperimentData data = filteredToExperiment(filtered);
    std::string kinematic_yaml = robot_utils::configPath(robot_dir, "kinematic_params.yaml");

    std::string robot_type = robot_name;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["robot_type"]) robot_type = kroot["robot_type"].as<std::string>();
    } catch (...) {}

    {
        std::string id_yaml = robot_utils::configPath(robot_dir, "inertia_identification.yaml");
        try {
            auto iroot = YAML::LoadFile(id_yaml);
            if (iroot["algorithm"])      algorithm      = iroot["algorithm"].as<int>();
            if (iroot["regularization"]) regularization = (iroot["regularization"].as<int>() != 0);
            if (iroot["damping"])        damping        = iroot["damping"].as<bool>();
        } catch (...) {
            // inertia_identification.yaml 不存在或损坏时使用默认值
        }
    }

    auto regressor = robot_dynamics::RegressorFactory::create(robot_type, kinematic_yaml);
    if (!regressor) { std::cerr << "无法创建回归器\n"; return 1; }

    auto flags = robot_dynamics::ParamFlags::NONE;
    if (damping)  flags = flags | robot_dynamics::ParamFlags::DAMPING;

    std::vector<std::string> algos;
    if (!algo_name.empty()) algos = {algo_name};
    else if (algorithm == 0) algos = BENCHMARK_ALGOS;
    else {
        auto it = ALGO_MAP.find(algorithm);
        if (it != ALGO_MAP.end()) algos = {it->second};
        else algos = {"IRLS"};
    }

    std::string output_yaml = robot_utils::resultInertiaPath(robot_dir,
        robot_name + "_inertia_identification.yaml");

    std::cout << "算法: ";
    for (auto& a : algos) std::cout << a << " ";
    std::cout << "\nDOF: " << regressor->nDof()
              << ", 参数数: " << regressor->numParameters(flags) << std::endl;

    bool append = false;
    for (const auto& algo : algos) {
        std::cout << "\n--- " << algo << " ---" << std::endl;
        auto r = runSingle(data, *regressor, flags, algo, regularization);
        std::cout << algo << " => RMSE: " << r.rmse
                  << " Nm, Max Error: " << r.max_err << " Nm" << std::endl;
        saveResults(output_yaml, algo, r.beta, r.rmse, r.max_err, robot_name, append,
                    regressor->nBodies(), regressor->nDof());
        append = true;
    }

    std::cout << "\n结果: " << output_yaml << std::endl;
    return 0;
}
