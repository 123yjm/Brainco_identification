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
    {9, "PCTIKHONOV"},
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
    bool armature = true;
    double pc_lambda = 1e-3;        // PC Tikhonov 正则化强度 (标量，向后兼容)
    double pc_lambda_mass = 0;     // m 的正则化权重 (0=使用 pc_lambda)
    double pc_lambda_com = 0;      // mx,my,mz 的正则化权重 (0=使用 pc_lambda)
    double pc_lambda_inertia = 0;  // Ixx..Izz 的正则化权重 (0=使用 pc_lambda)
};

void printHelp(const char* prog) {
    std::cout << "动力学参数辨识\n"
              << "用法: " << prog << " --robot <robot_dir> [选项]\n\n"
              << "选项:\n"
              << "  --robot <dir>       机器人目录 (如 robots/revoarm_right)\n"
              << "  --algo <name>       算法 (OLS/WLS/IRLS/TLS/EKF/NLS_FRICTION/PCTIKHONOV)\n"
              << "  --no-damping        禁用阻尼项辨识\n"
              << "  --no-armature       禁用转子惯量项辨识\n"
              << "  --pc-lambda <val>   PC 正则化强度 (默认 1e-3)\n"
              << "  --pc-lam-m <val>    质量 m 的正则化权重\n"
              << "  --pc-lam-c <val>    mx,my,mz 的正则化权重\n"
              << "  --pc-lam-i <val>    惯量 Ixx..Izz 的正则化权重\n"
              << "  --help              打印帮助信息\n";
}

// ---- PC (Physical Consistent) 辨识辅助函数 ---------------------------------

/// 从 inertia_prior.yaml 加载惯性先验参数，拼接为 (n_bodies*10) 维向量
Eigen::VectorXd loadInertiaPrior(const std::string& yaml_path,
                                  std::size_t n_bodies) {
    auto root = YAML::LoadFile(yaml_path);
    auto node = root["prior_inertia"];
    if (!node.IsSequence() || node.size() != n_bodies) {
        throw std::runtime_error(
            "inertia_prior.yaml: 期望 " + std::to_string(n_bodies) +
            " 个 body，实际 " + std::to_string(node.size()));
    }
    Eigen::VectorXd prior(static_cast<Eigen::Index>(n_bodies * 10));
    for (std::size_t body = 0; body < n_bodies; ++body) {
        auto row = node[body];
        if (!row.IsSequence() || row.size() != 10) {
            throw std::runtime_error(
                "inertia_prior.yaml: body " + std::to_string(body) +
                " 必须包含 10 个参数");
        }
        for (std::size_t i = 0; i < 10; ++i) {
            prior(static_cast<Eigen::Index>(body * 10 + i)) = row[i].as<double>();
        }
    }
    return prior;
}

/// 从摩擦辨识结果 YAML 加载 Fc, Fv 参数
/// 返回: [Fc0, Fv0, Fc1, Fv1, ..., Fc_{dof-1}, Fv_{dof-1}] (2*dof 维)
Eigen::VectorXd loadFrictionParams(const std::string& yaml_path,
                                    std::size_t dof) {
    auto root = YAML::LoadFile(yaml_path);
    auto node = root["parameters"];
    if (!node.IsSequence()) {
        throw std::runtime_error(
            "摩擦辨识结果 YAML: 'parameters' 必须是序列");
    }
    std::vector<double> vals;
    for (const auto& v : node) {
        vals.push_back(v.as<double>());
    }
    if (vals.size() != 2 * dof) {
        throw std::runtime_error(
            "摩擦辨识结果: 期望 " + std::to_string(2 * dof) +
            " 个参数，实际 " + std::to_string(vals.size()));
    }
    Eigen::VectorXd result(static_cast<Eigen::Index>(vals.size()));
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(vals.size()); ++i)
        result(i) = vals[static_cast<std::size_t>(i)];
    return result;
}

/// 计算激励轨迹每个时刻的摩擦力矩（堆叠格式，与 W 矩阵布局一致）
/// tau_f(k*dof + j) = Fc_j * sign(qd(k,j)) + Fv_j * qd(k,j)
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

/// PC 辨识专用运行函数：摩擦修正 + 先验中心 Tikhonov 正则化 (逐参数权重)
/// lambda_mass: m 的正则化权重
/// lambda_com:  mx, my, mz (质心位置) 的正则化权重
/// lambda_inertia: Ixx..Izz 的正则化权重
SingleResult runPCInertia(const ExperimentData& data,
                           const robot_dynamics::IDynamicsRegressor& regressor,
                           const Eigen::VectorXd& beta_prior,
                           const Eigen::VectorXd& friction_params,
                           double lambda_mass, double lambda_com, double lambda_inertia) {
    // PC 模式：仅惯性参数 (70 = 7×10，不含 armature/damping)
    auto flags = robot_dynamics::ParamFlags::NONE;
    const std::size_t num_params = regressor.numParameters(flags);
    const std::size_t dof = regressor.nDof();
    const Eigen::Index K = static_cast<Eigen::Index>(data.n_samples);
    const Eigen::Index rows_per = static_cast<Eigen::Index>(dof);

    // --- 构建观测矩阵 W (仅惯性参数) ---
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

    // --- 计算摩擦力矩并修正 ---
    Eigen::VectorXd tau_friction = computeFrictionTorque(data.qd, friction_params, dof);
    Eigen::VectorXd tau_corrected = tau_stacked - tau_friction;

    // --- 构建逐参数正则化权重向量 (3 层) ---
    // 每个 body: [0] m → λ_mass
    //           [1-3] mx, my, mz → λ_com
    //           [4-9] Ixx..Izz → λ_inertia
    Eigen::VectorXd reg_weights(static_cast<Eigen::Index>(num_params));
    const std::size_t n_bodies = regressor.nBodies();
    for (std::size_t b = 0; b < n_bodies; ++b) {
        Eigen::Index base = static_cast<Eigen::Index>(b * 10);
        reg_weights(base + 0) = lambda_mass;      // m
        reg_weights(base + 1) = lambda_com;       // mx
        reg_weights(base + 2) = lambda_com;       // my
        reg_weights(base + 3) = lambda_com;       // mz
        for (Eigen::Index i = 4; i < 10; ++i)    // Ixx..Izz
            reg_weights(base + i) = lambda_inertia;
    }

    // --- 先验中心 Tikhonov 求解 ---
    auto solver = std::make_unique<identification::PCTikhonov>();
    solver->setPrior(beta_prior);
    solver->setRegularizationWeights(reg_weights);
    Eigen::VectorXd beta = solver->solve(W_total, tau_corrected);

    // --- 对摩擦修正后的力矩计算残差 ---
    Eigen::VectorXd residual = W_total * beta - tau_corrected;
    double rmse = std::sqrt(residual.squaredNorm() / static_cast<double>(tau_corrected.size()));
    double max_err = residual.cwiseAbs().maxCoeff();

    return {"PCTIKHONOV", beta, rmse, max_err};
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

    // 分组：惯性 / armature / damping
    Eigen::Index inertial_count = static_cast<Eigen::Index>(n_bodies * 10);
    Eigen::Index n_dof_s = static_cast<Eigen::Index>(n_dof);
    Eigen::Index total = params.size();
    // remaining > n_dof → 同时有 armature 和 damping；否则只多一组
    bool has_two_groups = (total - inertial_count) > n_dof_s;

    // 惯性组（每行 10 个）
    for (Eigen::Index i = 0; i < inertial_count; i += 10) {
        out << "        ";
        for (Eigen::Index j = 0; j < 10 && i + j < inertial_count; ++j) {
            out << params(i + j);
            if (i + j + 1 < total) out << ",";
        }
        out << "\n";
    }

    if (total > inertial_count) {
        out << "\n";  // 惯性参数与下一组之间空行

        if (has_two_groups) {
            // armature 组
            out << "        ";
            Eigen::Index arm_start = inertial_count;
            Eigen::Index arm_end = inertial_count + n_dof_s;
            for (Eigen::Index j = arm_start; j < arm_end; ++j) {
                out << params(j);
                if (j + 1 < total) out << ",";
            }
            out << "\n\n";  // armature 与 damping 之间空行

            // damping 组
            out << "        ";
            for (Eigen::Index j = arm_end; j < total; ++j) {
                out << params(j);
                if (j + 1 < total) out << ",";
            }
            out << "\n";
        } else {
            // 仅 armature 或仅 damping
            out << "        ";
            for (Eigen::Index j = inertial_count; j < total; ++j) {
                out << params(j);
                if (j + 1 < total) out << ",";
            }
            out << "\n";
        }
    }

    out << "        ]\n";
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

    // 从 inertia_identification.yaml 读取 armature / damping / algorithm 默认值
    {
        std::string id_yaml = robot_utils::configPath(opt.robot_dir, "inertia_identification.yaml");
        try {
            auto iroot = YAML::LoadFile(id_yaml);
            if (iroot["algorithm"])      opt.algorithm      = iroot["algorithm"].as<int>();
            if (iroot["regularization"]) opt.regularization = (iroot["regularization"].as<int>() != 0);
            if (iroot["armature"])       opt.armature       = iroot["armature"].as<bool>();
            if (iroot["damping"])        opt.damping        = iroot["damping"].as<bool>();
            if (iroot["pc_lambda"])        opt.pc_lambda        = iroot["pc_lambda"].as<double>();
            if (iroot["pc_lambda_mass"])    opt.pc_lambda_mass    = iroot["pc_lambda_mass"].as<double>();
            if (iroot["pc_lambda_com"])     opt.pc_lambda_com     = iroot["pc_lambda_com"].as<double>();
            if (iroot["pc_lambda_inertia"]) opt.pc_lambda_inertia = iroot["pc_lambda_inertia"].as<double>();
        } catch (...) {
            // inertia_identification.yaml 不存在或损坏时使用默认值
        }
    }

    // ---- 第二次扫描：CLI 覆盖（优先级高于 YAML）----------------------------
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--algo" && i + 1 < argc) opt.algo_name = argv[++i];
        else if (arg == "--no-damping") opt.damping = false;
        else if (arg == "--no-armature") opt.armature = false;
        else if (arg == "--pc-lambda" && i + 1 < argc)
            opt.pc_lambda = std::stod(argv[++i]);
        else if (arg == "--pc-lam-m" && i + 1 < argc)
            opt.pc_lambda_mass = std::stod(argv[++i]);
        else if (arg == "--pc-lam-c" && i + 1 < argc)
            opt.pc_lambda_com = std::stod(argv[++i]);
        else if (arg == "--pc-lam-i" && i + 1 < argc)
            opt.pc_lambda_inertia = std::stod(argv[++i]);
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
    if (opt.armature) flags = flags | robot_dynamics::ParamFlags::ARMATURE;
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

    // ---- PC 辨识模式：加载先验和摩擦参数 ------------------------------------
    Eigen::VectorXd beta_prior;       // 仅 PC 模式使用
    Eigen::VectorXd friction_params;  // 仅 PC 模式使用
    bool pc_mode = (!algos.empty() && algos[0] == "PCTIKHONOV");

    if (pc_mode) {
        // 检查先验文件
        std::string prior_file = robot_utils::configPath(opt.robot_dir, "inertia_prior.yaml");
        if (!std::filesystem::exists(prior_file)) {
            std::cerr << "错误: PC 辨识需要惯性先验文件: " << prior_file
                      << "\n请创建 robots/<robot>/config/inertia_prior.yaml\n";
            return 1;
        }

        // 检查摩擦辨识结果
        std::string friction_file = robot_utils::resultFrictionPath(
            opt.robot_dir, opt.robot_name + "_friction_identification.yaml");
        if (!std::filesystem::exists(friction_file)) {
            std::cerr << "错误: PC 辨识需要摩擦力辨识结果: " << friction_file
                      << "\n请先运行 identify_friction --robot " << opt.robot_dir << "\n";
            return 1;
        }

        // 加载
        try {
            beta_prior = loadInertiaPrior(prior_file, regressor->nBodies());
            friction_params = loadFrictionParams(friction_file, regressor->nDof());
        } catch (const std::exception& e) {
            std::cerr << "加载 PC 辨识数据失败: " << e.what() << std::endl;
            return 1;
        }

        // PC 模式强制仅惯性参数
        flags = robot_dynamics::ParamFlags::NONE;
        if (opt.armature || opt.damping) {
            std::cout << "注意: PC 辨识仅支持惯性参数，armature/damping 已忽略\n";
        }

        // 打印信息 — 显示三层权重
        double lam_m = (opt.pc_lambda_mass > 0) ? opt.pc_lambda_mass : opt.pc_lambda;
        double lam_c = (opt.pc_lambda_com > 0) ? opt.pc_lambda_com : opt.pc_lambda;
        double lam_i = (opt.pc_lambda_inertia > 0) ? opt.pc_lambda_inertia : opt.pc_lambda;
        std::cout << "PC 先验文件: " << prior_file
                  << "\n摩擦参数文件: " << friction_file
                  << "\nλ_m=" << lam_m << " λ_com=" << lam_c << " λ_I=" << lam_i
                  << ", 先验维度 = " << beta_prior.size() << std::endl;
    }

    std::cout << "算法: ";
    for (auto& a : algos) std::cout << a << " ";
    std::cout << "\nDOF: " << regressor->nDof()
              << ", 参数数: " << regressor->numParameters(flags)
              << ", armature=" << (opt.armature ? "yes" : "no")
              << ", damping=" << (opt.damping ? "yes" : "no") << std::endl;

    bool append = false;
    for (const auto& algo : algos) {
        std::cout << "\n--- " << algo << " ---" << std::endl;

        SingleResult r;
        if (algo == "PCTIKHONOV") {
            double lam_m = (opt.pc_lambda_mass > 0) ? opt.pc_lambda_mass : opt.pc_lambda;
            double lam_c = (opt.pc_lambda_com > 0) ? opt.pc_lambda_com : opt.pc_lambda;
            double lam_i = (opt.pc_lambda_inertia > 0) ? opt.pc_lambda_inertia : opt.pc_lambda;
            r = runPCInertia(data, *regressor, beta_prior, friction_params, lam_m, lam_c, lam_i);
        } else {
            r = runSingle(data, *regressor, flags, algo, opt.regularization);
        }

        std::cout << algo << " => RMSE: " << r.rmse
                  << " Nm, Max Error: " << r.max_err << " Nm" << std::endl;
        saveResults(opt.output_file, algo, r.beta, r.rmse, r.max_err, opt.robot_name, append,
                    regressor->nBodies(), regressor->nDof());
        append = true;
    }

    std::cout << "\n结果: " << opt.output_file << std::endl;
    return 0;
}
