/**
 * @file main_solve_inertia_PC.cpp
 * @brief PC (Physical Consistent) 惯量辨识独立入口
 *
 * 用法: ./identify_inertia_PC -r <robot> [--pc-lam-m <val>] [--pc-lam-c <val>] [--pc-lam-i <val>]
 *
 * 自动读取 config/inertia_PC_identification.yaml (先验 + 正则化参数),
 * 加载摩擦辨识结果, 用先验中心 Tikhonov 正则化求解 70 个惯性参数,
 * 输出到 result_inertia_PC/<robot>_inertia_identification.yaml
 */

#include "algorithms.hpp"
#include "data_loader.hpp"
#include "regressor_factory.hpp"
#include "robot_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

const double kDefaultLambdaMass    = 0.1;
const double kDefaultLambdaCom     = 1e6;
const double kDefaultLambdaInertia = 5e7;

void printHelp(const char* prog) {
    std::cout << "PC (Physical Consistent) 惯量辨识\n"
              << "用法: " << prog << " -r <robot> [选项]\n\n"
              << "选项:\n"
              << "  -r, --robot <name>   机器人名称 (如 revoarm_right)\n"
              << "  --pc-lam-m <val>     质量 m 的正则化权重 (默认 " << kDefaultLambdaMass << ")\n"
              << "  --pc-lam-c <val>     mx,my,mz 的正则化权重 (默认 " << kDefaultLambdaCom << ")\n"
              << "  --pc-lam-i <val>     惯量 Ixx..Izz 的正则化权重 (默认 " << kDefaultLambdaInertia << ")\n"
              << "  -h, --help           打印帮助信息\n";
}

// =============================================================================
// 辅助函数
// =============================================================================

Eigen::VectorXd loadInertiaPrior(const std::string& yaml_path,
                                  std::size_t n_bodies) {
    auto root = YAML::LoadFile(yaml_path);
    auto node = root["prior_inertia"];
    if (!node.IsSequence() || node.size() != n_bodies) {
        throw std::runtime_error(
            "inertia_PC_identification.yaml: 期望 " + std::to_string(n_bodies) +
            " 个 body，实际 " + std::to_string(node.size()));
    }
    Eigen::VectorXd prior(static_cast<Eigen::Index>(n_bodies * 10));
    for (std::size_t body = 0; body < n_bodies; ++body) {
        auto row = node[body];
        if (!row.IsSequence() || row.size() != 10) {
            throw std::runtime_error(
                "inertia_PC_identification.yaml: body " + std::to_string(body) +
                " 必须包含 10 个参数");
        }
        for (std::size_t i = 0; i < 10; ++i)
            prior(static_cast<Eigen::Index>(body * 10 + i)) = row[i].as<double>();
    }
    return prior;
}

Eigen::VectorXd loadFrictionParams(const std::string& yaml_path,
                                    std::size_t dof) {
    auto root = YAML::LoadFile(yaml_path);
    auto node = root["parameters"];
    if (!node.IsSequence())
        throw std::runtime_error("摩擦辨识结果: 'parameters' 必须是序列");
    std::vector<double> vals;
    for (const auto& v : node) vals.push_back(v.as<double>());
    if (vals.size() != 2 * dof)
        throw std::runtime_error(
            "摩擦辨识结果: 期望 " + std::to_string(2 * dof) +
            " 个参数，实际 " + std::to_string(vals.size()));
    Eigen::VectorXd result(static_cast<Eigen::Index>(vals.size()));
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(vals.size()); ++i)
        result(i) = vals[static_cast<std::size_t>(i)];
    return result;
}

Eigen::VectorXd computeFrictionTorque(const Eigen::MatrixXd& qd,
                                       const Eigen::VectorXd& friction_params,
                                       std::size_t dof) {
    const Eigen::Index K = qd.rows();
    const Eigen::Index D = static_cast<Eigen::Index>(dof);
    Eigen::VectorXd tau_f(K * D);
    for (Eigen::Index k = 0; k < K; ++k) {
        for (Eigen::Index j = 0; j < D; ++j) {
            double dq = qd(k, j);
            double sign_dq = (dq >= 0.0) ? 1.0 : -1.0;
            double Fc = friction_params(2 * j);
            double Fv = friction_params(2 * j + 1);
            tau_f(k * D + j) = Fc * sign_dq + Fv * dq;
        }
    }
    return tau_f;
}

struct SingleResult {
    std::string algo;
    Eigen::VectorXd beta;
    double rmse;
    double max_err;
};

SingleResult runPCInertia(const ExperimentData& data,
                           const robot_dynamics::IDynamicsRegressor& regressor,
                           const Eigen::VectorXd& beta_prior,
                           const Eigen::VectorXd& friction_params,
                           double lambda_mass, double lambda_com, double lambda_inertia) {
    auto flags = robot_dynamics::ParamFlags::NONE;
    const std::size_t num_params = regressor.numParameters(flags);
    const std::size_t dof = regressor.nDof();
    const Eigen::Index K = static_cast<Eigen::Index>(data.n_samples);
    const Eigen::Index rows_per = static_cast<Eigen::Index>(dof);

    // 构建观测矩阵 W (仅惯性参数)
    Eigen::MatrixXd W_total = Eigen::MatrixXd::Zero(
        rows_per * K, static_cast<Eigen::Index>(num_params));
    Eigen::VectorXd tau_stacked(rows_per * K);

    for (Eigen::Index k = 0; k < K; ++k) {
        Eigen::VectorXd q   = data.q.row(k).transpose();
        Eigen::VectorXd qd  = data.qd.row(k).transpose();
        Eigen::VectorXd qdd = data.qdd.row(k).transpose();
        Eigen::MatrixXd Y_k = regressor.computeRegressorMatrix(q, qd, qdd, flags);
        W_total.block(k * rows_per, 0, rows_per,
                      static_cast<Eigen::Index>(num_params)) = Y_k;
        for (Eigen::Index j = 0; j < rows_per; ++j)
            tau_stacked(k * rows_per + j) = data.tau(k, j);
    }

    // 摩擦修正
    Eigen::VectorXd tau_friction = computeFrictionTorque(data.qd, friction_params, dof);
    Eigen::VectorXd tau_corrected = tau_stacked - tau_friction;

    // 三层正则化权重
    Eigen::VectorXd reg_weights(static_cast<Eigen::Index>(num_params));
    const std::size_t n_bodies = regressor.nBodies();
    for (std::size_t b = 0; b < n_bodies; ++b) {
        Eigen::Index base = static_cast<Eigen::Index>(b * 10);
        reg_weights(base + 0) = lambda_mass;
        reg_weights(base + 1) = lambda_com;
        reg_weights(base + 2) = lambda_com;
        reg_weights(base + 3) = lambda_com;
        for (Eigen::Index i = 4; i < 10; ++i)
            reg_weights(base + i) = lambda_inertia;
    }

    // 求解
    auto solver = std::make_unique<identification::PCTikhonov>();
    solver->setPrior(beta_prior);
    solver->setRegularizationWeights(reg_weights);
    Eigen::VectorXd beta = solver->solve(W_total, tau_corrected);

    Eigen::VectorXd residual = W_total * beta - tau_corrected;
    double rmse = std::sqrt(residual.squaredNorm() / static_cast<double>(tau_corrected.size()));
    double max_err = residual.cwiseAbs().maxCoeff();

    return {"PCTIKHONOV", beta, rmse, max_err};
}

void saveResults(const std::string& path, const std::string& algo_name,
                 const Eigen::VectorXd& params,
                 double rmse, double max_error,
                 const std::string& robot_name,
                 std::size_t n_bodies, std::size_t n_dof) {
    std::filesystem::path p(path);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    std::ofstream out(path);
    if (!out) { std::cerr << "无法写入: " << path << std::endl; return; }
    std::time_t now = std::time(nullptr);
    char buf[64]; std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    out << std::scientific << std::setprecision(15);
    out << "calibration_date: \"" << buf << "\"\n"
        << "robot: \"" << robot_name << "\"\n"
        << "evaluation_method: \"torque_residual_rmse\"\n"
        << "benchmark_results:\n";
    out << "  - algorithm: \"" << algo_name << "\"\n"
        << "    torque_rmse: " << rmse << "\n"
        << "    torque_max_error: " << max_error << "\n"
        << "    parameters:\n"
        << "        [\n";

    Eigen::Index inertial_count = static_cast<Eigen::Index>(n_bodies * 10);
    for (Eigen::Index i = 0; i < inertial_count; i += 10) {
        out << "        ";
        for (Eigen::Index j = 0; j < 10 && i + j < inertial_count; ++j) {
            out << params(i + j);
            if (i + j + 1 < params.size()) out << ",";
        }
        out << "\n";
    }

    Eigen::Index total = params.size();
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

}  // anonymous namespace

int main(int argc, char* argv[]) {
    // ---- CLI 解析 ----
    std::string robot_name;
    double lam_m = kDefaultLambdaMass;
    double lam_c = kDefaultLambdaCom;
    double lam_i = kDefaultLambdaInertia;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") { printHelp(argv[0]); return 0; }
        else if ((arg == "-r" || arg == "--robot") && i + 1 < argc)
            robot_name = argv[++i];
        else if (arg == "--pc-lam-m" && i + 1 < argc)
            lam_m = std::stod(argv[++i]);
        else if (arg == "--pc-lam-c" && i + 1 < argc)
            lam_c = std::stod(argv[++i]);
        else if (arg == "--pc-lam-i" && i + 1 < argc)
            lam_i = std::stod(argv[++i]);
    }

    if (robot_name.empty()) {
        std::cerr << "错误: 需要 -r <robot>\n";
        return 1;
    }

    std::string robot_dir = robot_utils::resolvePath(
        robot_utils::resolveRobotDir(robot_name), PROJECT_ROOT_DIR);

    // ---- 路径推导 ----
    std::string robot_name_short = robot_utils::robotNameFromDir(robot_dir);
    std::string kinematic_yaml  = robot_utils::configPath(robot_dir, "kinematic_params.yaml");
    std::string pc_config_yaml  = robot_utils::configPath(robot_dir, "inertia_PC_identification.yaml");
    std::string data_file       = robot_utils::resultInertiaPath(robot_dir,
        robot_name_short + "_filtered_data.csv");
    std::string friction_file   = robot_utils::resultFrictionPath(robot_dir,
        robot_name_short + "_friction_identification.yaml");
    std::string output_file     = robot_utils::resultInertiaPCPath(robot_dir,
        robot_name_short + "_inertia_identification.yaml");

    // ---- 加载机器人模型 ----
    std::string robot_type = robot_name_short;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["robot_type"]) robot_type = kroot["robot_type"].as<std::string>();
    } catch (...) {}

    std::cout << "机器人: " << robot_name_short << " (type: " << robot_type << ")"
              << "\nPC 配置: " << pc_config_yaml << std::endl;

    auto regressor = robot_dynamics::RegressorFactory::create(robot_type, kinematic_yaml);

    // ---- 加载数据 ----
    std::cout << "加载数据: " << data_file << std::endl;
    ExperimentData data;
    try {
        data = DataLoader::loadCSV(data_file, regressor->nDof());
    } catch (const std::exception& e) {
        std::cerr << "加载失败: " << e.what() << std::endl;
        return 1;
    }

    // ---- 加载 PC 配置 (先验 + λ 覆盖) ----
    if (!std::filesystem::exists(pc_config_yaml)) {
        std::cerr << "错误: PC 配置文件不存在: " << pc_config_yaml << std::endl;
        return 1;
    }
    try {
        auto pc_root = YAML::LoadFile(pc_config_yaml);
        // CLI 参数优先于配置文件（CLI 未设置时用默认值，配置文件可覆盖默认值）
        // 若用户未通过 CLI 指定（值仍为默认），则从配置文件读取
        if (pc_root["pc_lambda_mass"])
            lam_m = pc_root["pc_lambda_mass"].as<double>();
        if (pc_root["pc_lambda_com"])
            lam_c = pc_root["pc_lambda_com"].as<double>();
        if (pc_root["pc_lambda_inertia"])
            lam_i = pc_root["pc_lambda_inertia"].as<double>();
    } catch (...) {}

    Eigen::VectorXd beta_prior;
    try {
        beta_prior = loadInertiaPrior(pc_config_yaml, regressor->nBodies());
    } catch (const std::exception& e) {
        std::cerr << "加载先验失败: " << e.what() << std::endl;
        return 1;
    }

    // ---- 加载摩擦参数 ----
    if (!std::filesystem::exists(friction_file)) {
        std::cerr << "错误: 摩擦辨识结果不存在: " << friction_file
                  << "\n请先运行 identify_friction\n";
        return 1;
    }
    Eigen::VectorXd friction_params;
    try {
        friction_params = loadFrictionParams(friction_file, regressor->nDof());
    } catch (const std::exception& e) {
        std::cerr << "加载摩擦参数失败: " << e.what() << std::endl;
        return 1;
    }

    // ---- 运行 ----
    std::cout << "λ_m=" << lam_m << " λ_com=" << lam_c << " λ_I=" << lam_i
              << "\n先验维度 = " << beta_prior.size()
              << "\nDOF: " << regressor->nDof()
              << ", 参数数: " << regressor->numParameters(robot_dynamics::ParamFlags::NONE)
              << "\n\n--- PCTIKHONOV ---" << std::endl;

    auto r = runPCInertia(data, *regressor, beta_prior, friction_params, lam_m, lam_c, lam_i);

    std::cout << "PCTIKHONOV => RMSE: " << r.rmse
              << " Nm, Max Error: " << r.max_err << " Nm" << std::endl;

    saveResults(output_file, "PCTIKHONOV", r.beta, r.rmse, r.max_err,
                robot_name_short, regressor->nBodies(), regressor->nDof());

    std::cout << "\n结果: " << output_file << std::endl;
    return 0;
}
