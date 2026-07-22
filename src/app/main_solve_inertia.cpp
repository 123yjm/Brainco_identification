/**
 * @file main_solve.cpp
 * @brief 动力学参数辨识入口
 *
 * 用法: ./identify --robot <robot_dir> [--algo <name>] [--no-damping] [--help]
 */

#include "algorithms.hpp"
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

struct SolveOptions {
    std::string robot_dir;
    std::string robot_name;
    std::string kinematic_params;
    std::string data_file;
    std::string output_file;
    int algorithm = 3;
    std::string algo_name;
    bool regularization = true;
    bool damping = true;
};

void printHelp(const char* prog) {
    std::cout << "动力学参数辨识\n"
              << "用法: " << prog << " --robot <robot_dir> [选项]\n\n"
              << "选项:\n"
              << "  --robot <dir>       机器人目录 (如 robots/revoarm_right)\n"
              << "  --algo <name>       算法 (OLS/WLS/IRLS/TLS/EKF/NLS_FRICTION)\n"
              << "  --no-damping        禁用阻尼项辨识\n"
              << "  --help              打印帮助信息\n";
}

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

void writeIdentificationCsv(const std::string& path,
                            const ExperimentData& data,
                            const robot_dynamics::IDynamicsRegressor& regressor,
                            const Eigen::VectorXd& beta,
                            robot_dynamics::ParamFlags flags) {
    std::filesystem::path p(path);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    std::ofstream out(path);
    if (!out) { std::cerr << "无法写入: " << path << std::endl; return; }

    const std::size_t dof = regressor.nDof();
    const Eigen::Index K = static_cast<Eigen::Index>(data.n_samples);
    const bool has_tau_raw = (data.tau_raw.size() > 0);

    // Write header
    out << "time";
    for (std::size_t j = 0; j < dof; ++j) out << ",q" << j;
    for (std::size_t j = 0; j < dof; ++j) out << ",qd" << j;
    for (std::size_t j = 0; j < dof; ++j) out << ",qdd" << j;
    for (std::size_t j = 0; j < dof; ++j) out << ",tau" << j;
    for (std::size_t j = 0; j < dof; ++j) out << ",tau_raw" << j;
    for (std::size_t j = 0; j < dof; ++j) out << ",tau_estimate" << j;
    out << "\n";

    out << std::fixed << std::setprecision(6);
    for (Eigen::Index k = 0; k < K; ++k) {
        out << data.time[k];

        // q
        for (std::size_t j = 0; j < dof; ++j) out << "," << data.q(k, j);
        // qd
        for (std::size_t j = 0; j < dof; ++j) out << "," << data.qd(k, j);
        // qdd
        for (std::size_t j = 0; j < dof; ++j) out << "," << data.qdd(k, j);
        // tau (filtered)
        for (std::size_t j = 0; j < dof; ++j) out << "," << data.tau(k, j);
        // tau_raw
        for (std::size_t j = 0; j < dof; ++j) {
            out << ",";
            if (has_tau_raw) out << data.tau_raw(k, j);
            else out << "0.0";
        }

        // tau_estimate = Y(q,qd,qdd) * beta
        Eigen::VectorXd q   = data.q.row(k).transpose();
        Eigen::VectorXd qd  = data.qd.row(k).transpose();
        Eigen::VectorXd qdd = data.qdd.row(k).transpose();
        Eigen::MatrixXd Y_k = regressor.computeRegressorMatrix(q, qd, qdd, flags);
        Eigen::VectorXd tau_est = Y_k * beta;
        for (std::size_t j = 0; j < dof; ++j) out << "," << tau_est(j);

        out << "\n";
    }

    std::cout << "辨识结果 CSV 已保存: " << path << " (" << K << " 行)" << std::endl;
}

}  // anonymous namespace

int main(int argc, char* argv[]) {
    // ---- 第一次扫描：仅解析 --robot / --help --------------------------------
    SolveOptions opt;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") { printHelp(argv[0]); return 0; }
        else if ((arg == "--robot" || arg == "-r") && i + 1 < argc)
            opt.robot_dir = robot_utils::resolvePath(
                robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
    }

    if (opt.robot_dir.empty()) {
        std::cerr << "错误: 需要 --robot <dir>\n";
        return 1;
    }

    opt.robot_name = robot_utils::robotNameFromDir(opt.robot_dir);
    opt.kinematic_params = robot_utils::configPath(opt.robot_dir, "kinematic_params.yaml");
    opt.data_file = robot_utils::resultInertiaPath(opt.robot_dir, opt.robot_name + "_filtered_data.csv");
    opt.output_file = robot_utils::resultInertiaPath(opt.robot_dir, opt.robot_name + "_inertia_identification.yaml");

    // 从 kinematic_params.yaml 读取 robot_type
    std::string robot_type = opt.robot_name;  // fallback
    try {
        auto kroot = YAML::LoadFile(opt.kinematic_params);
        if (kroot["robot_type"]) robot_type = kroot["robot_type"].as<std::string>();
    } catch (...) {}

    {
        std::string id_yaml = robot_utils::configPath(opt.robot_dir, "inertia_identification.yaml");
        try {
            auto iroot = YAML::LoadFile(id_yaml);
            if (iroot["algorithm"])      opt.algorithm      = iroot["algorithm"].as<int>();
            if (iroot["regularization"]) opt.regularization = (iroot["regularization"].as<int>() != 0);
            if (iroot["damping"])        opt.damping        = iroot["damping"].as<bool>();
        } catch (...) {
            // inertia_identification.yaml 不存在或损坏时使用默认值
        }
    }

    // ---- 第二次扫描：CLI 覆盖（优先级高于 YAML）----------------------------
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--algo" && i + 1 < argc) opt.algo_name = argv[++i];
        else if (arg == "--no-damping") opt.damping = false;
    }

    // ---- 加载机器人模型 ---------------------------------------------------
    std::cout << "机器人: " << opt.robot_name << " (type: " << robot_type << ")"
              << "\n运动学: " << opt.kinematic_params << std::endl;
    std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor;
    try {
        regressor = robot_dynamics::RegressorFactory::create(
            robot_type, opt.kinematic_params);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    // ---- 加载数据 ---------------------------------------------------------
    std::cout << "加载数据: " << opt.data_file << std::endl;
    ExperimentData data;
    try {
        data = DataLoader::loadCSV(opt.data_file, regressor->nDof());
    } catch (const std::exception& e) {
        std::cerr << "加载失败: " << e.what() << std::endl;
        return 1;
    }

    // ---- flags ------------------------------------------------------------
    auto flags = robot_dynamics::ParamFlags::NONE;
    if (opt.damping)  flags = flags | robot_dynamics::ParamFlags::DAMPING;

    // ---- 运行 -------------------------------------------------------------
    std::vector<std::string> algos;
    if (!opt.algo_name.empty()) algos = {opt.algo_name};
    else if (opt.algorithm == 0) algos = BENCHMARK_ALGOS;
    else {
        auto it = ALGO_MAP.find(opt.algorithm);
        if (it != ALGO_MAP.end()) algos = {it->second};
        else { std::cerr << "无效算法编号: " << opt.algorithm << std::endl; return 1; }
    }

    std::cout << "算法: ";
    for (auto& a : algos) std::cout << a << " ";
    std::cout << "\nDOF: " << regressor->nDof()
              << ", 参数数: " << regressor->numParameters(flags)
              << ", damping=" << (opt.damping ? "yes" : "no") << std::endl;

    bool append = false;
    for (const auto& algo : algos) {
        std::cout << "\n--- " << algo << " ---" << std::endl;
        auto r = runSingle(data, *regressor, flags, algo, opt.regularization);
        std::cout << algo << " => RMSE: " << r.rmse
                  << " Nm, Max Error: " << r.max_err << " Nm" << std::endl;
        saveResults(opt.output_file, algo, r.beta, r.rmse, r.max_err, opt.robot_name, append,
                    regressor->nBodies(), regressor->nDof());
        append = true;

        // Write per-sample identification result CSV
        std::string csv_path = robot_utils::resultInertiaPath(
            opt.robot_dir, opt.robot_name + "_identification_result_" + algo + ".csv");
        writeIdentificationCsv(csv_path, data, *regressor, r.beta, flags);
    }

    std::cout << "\n结果: " << opt.output_file << std::endl;
    return 0;
}
