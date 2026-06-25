/**
 * @file main_solve_friction.cpp
 * @brief 摩擦力辨识 — Coulomb + Viscous 摩擦系数求解
 *
 * 用法: ./identify_friction --robot <robot_dir>
 *
 * 从匀速段数据中逐点扣除重力（使用已辨识的惯性参数 beta），
 * 对每个关节独立做全数据点线性回归求解 Fc, Fv。
 *
 * 摩擦模型: tau_friction = Fc * sign(dq) + Fv * dq
 * 重力扣除: tau_ng = tau - Y(q, 0, 0) * beta_inertial
 */

#include "data_loader.hpp"
#include "regressor_factory.hpp"
#include "robot_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/QR>

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

void printHelp(const char* prog) {
    std::cout
        << "摩擦力辨识 — Coulomb + Viscous\n\n"
        << "用法: " << prog << " --robot <robot_dir>\n\n"
        << "选项:\n"
        << "  --robot <dir>  机器人目录 (如 robots/revoarm_right)\n"
        << "  --help         打印帮助信息\n\n"
        << "使用已辨识的惯性参数 beta 逐点扣除重力，\n"
        << "对每个关节做全数据点 OLS 回归求解 Fc, Fv。\n";
}

Eigen::VectorXd parseVector(const YAML::Node& node) {
    Eigen::VectorXd v(node.size());
    for (std::size_t i = 0; i < node.size(); ++i)
        v(static_cast<Eigen::Index>(i)) = node[i].as<double>();
    return v;
}

void printSep() { std::cout << std::string(55, '-') << "\n"; }

}  // anonymous namespace

int main(int argc, char* argv[]) {
    // ---- CLI ----------------------------------------------------------------
    std::string robot_dir;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") { printHelp(argv[0]); return 0; }
        else if ((arg == "--robot" || arg == "-r") && i + 1 < argc)
            robot_dir = robot_utils::resolvePath(
                robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
    }
    if (robot_dir.empty()) { std::cerr << "错误: 需要 --robot <dir>\n"; return 1; }

    std::string robot_name       = robot_utils::robotNameFromDir(robot_dir);
    std::string friction_yaml    = robot_utils::configPath(robot_dir, "friction_trajectory.yaml");
    std::string friction_id_yaml = robot_utils::configPath(robot_dir, "friction_identification.yaml");
    std::string kinematic_yaml   = robot_utils::configPath(robot_dir, "kinematic_params.yaml");
    std::string beta_yaml        = robot_utils::resultInertiaPath(robot_dir,
        robot_name + "_inertia_identification.yaml");

    // ---- kinematic_params.yaml ----------------------------------------------
    std::size_t dof = 0;
    std::string robot_type = robot_name;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["dof"]) dof = kroot["dof"].as<std::size_t>();
        if (kroot["robot_type"]) robot_type = kroot["robot_type"].as<std::string>();
    } catch (const std::exception& e) {
        std::cerr << "错误: kinematic_params.yaml — " << e.what() << "\n"; return 1;
    }

    // ---- friction_trajectory.yaml -------------------------------------------
    double a, v1, v2;
    Eigen::VectorXd q_init, q_target_v1, q_target_v2;
    try {
        auto root = YAML::LoadFile(friction_yaml);
        a           = root["acceleration"].as<double>();
        v1          = root["v1"].as<double>();
        v2          = root["v2"].as<double>();
        q_init      = parseVector(root["q_init"]);
        q_target_v1 = parseVector(root["q_target_v1"]);
        q_target_v2 = parseVector(root["q_target_v2"]);
    } catch (const std::exception& e) {
        std::cerr << "错误: friction_trajectory.yaml — " << e.what() << "\n"; return 1;
    }

    // ---- friction_identification.yaml ---------------------------------------
    double interval_shrink = 1.0;
    try {
        auto froot = YAML::LoadFile(friction_id_yaml);
        if (froot["interval_shrink"])
            interval_shrink = froot["interval_shrink"].as<double>();
    } catch (...) {}

    // ---- 查找数据 CSV -------------------------------------------------------
    std::string data_file = robot_utils::findFirstFile(
        robot_utils::dataFrictionPath(robot_dir, ""), "*.csv");
    if (data_file.empty()) {
        std::cerr << "错误: data_friction 下未找到 .csv\n"; return 1;
    }

    // ---- 加载数据 -----------------------------------------------------------
    ExperimentData data;
    try { data = DataLoader::loadCSV(data_file, dof); }
    catch (const std::exception& e) {
        std::cerr << "错误: 数据加载失败 — " << e.what() << "\n"; return 1;
    }

    // ---- 创建 regressor -----------------------------------------------------
    std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor;
    try {
        regressor = robot_dynamics::RegressorFactory::create(robot_type, kinematic_yaml);
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法创建 regressor — " << e.what() << "\n"; return 1;
    }

    // ---- 加载惯性 beta -------------------------------------------------------
    Eigen::VectorXd beta_inertial;
    try {
        auto root = YAML::LoadFile(beta_yaml);
        auto params_node = root["benchmark_results"][0]["parameters"];
        std::vector<double> vals;
        for (const auto& item : params_node) {
            if (item.IsSequence())
                for (const auto& v : item) vals.push_back(v.as<double>());
            else
                vals.push_back(item.as<double>());
        }
        beta_inertial = Eigen::VectorXd(vals.size());
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(vals.size()); ++i)
            beta_inertial(i) = vals[i];
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法加载 beta — " << e.what() << "\n"; return 1;
    }

    // ---- 计算各关节匀速段时间窗口 -------------------------------------------
    struct Window { double t_start, t_end; int joint; };

    std::vector<Window> windows;
    double t_global = 0.0;

    for (std::size_t j = 0; j < dof; ++j) {
        double q_now = q_init(j);
        double targets[4] = { q_target_v1(j), q_init(j), q_target_v2(j), q_init(j) };
        double speeds[4]  = { v1, v1, v2, v2 };

        for (int s = 0; s < 4; ++s) {
            double q_goal = targets[s], v = speeds[s];
            double distance = std::abs(q_goal - q_now);
            double t_const = 0.0, t_acc = 0.0;
            if (distance > 1e-12) {
                double d_acc = v * v / (2.0 * a);
                t_acc = v / a;
                if (2.0 * d_acc <= distance)
                    t_const = (distance - 2.0 * d_acc) / v;
                else { t_acc = std::sqrt(distance / a); t_const = 0.0; }
            }
            double t_total = 2.0 * t_acc + t_const;
            if (t_const > 0.0) {
                double center = t_global + t_acc + t_const / 2.0;
                double half   = t_const / 2.0 * interval_shrink;
                windows.push_back({center - half, center + half,
                                   static_cast<int>(j)});
            }
            t_global += t_total;
            q_now = q_goal;
        }
    }

    // ---- 打印配置 -----------------------------------------------------------
    std::cout << "═══════════════════════════════════════════\n"
              << "  摩擦力辨识 — Coulomb + Viscous\n"
              << "  (重力精确扣除 + 全数据点回归)\n"
              << "═══════════════════════════════════════════\n"
              << "机器人:           " << robot_name << "\n"
              << "数据:             " << data_file << "\n"
              << "beta:             " << beta_yaml << "\n"
              << "样本数:           " << data.n_samples << "\n"
              << "v1 / v2:          " << v1 << " / " << v2 << " rad/s\n"
              << "interval_shrink:  " << interval_shrink << "\n\n";

    // ---- 逐关节：收集全部匀速段数据点，扣除重力，OLS 回归 ------------------
    Eigen::VectorXd zeros = Eigen::VectorXd::Zero(dof);
    auto flags = robot_dynamics::ParamFlags::ALL;

    std::vector<double> fc_results(dof), fv_results(dof), rmse_results(dof);

    for (std::size_t j = 0; j < dof; ++j) {
        std::vector<double> Y_vec, W1_vec, W2_vec;

        for (auto& w : windows) {
            if (w.joint != static_cast<int>(j)) continue;

            for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(data.n_samples); ++k) {
                if (data.time[k] < w.t_start || data.time[k] > w.t_end) continue;

                double tau_k = data.tau(k, static_cast<Eigen::Index>(j));
                double dq_k  = data.qd(k, static_cast<Eigen::Index>(j));

                // 重力扣除: tau_ng = tau - Y(q, 0, 0) * beta
                Eigen::VectorXd q_k = data.q.row(k).transpose();
                Eigen::MatrixXd Y_grav = regressor->computeRegressorMatrix(q_k, zeros, zeros, flags);
                Eigen::VectorXd tau_grav_vec = Y_grav * beta_inertial;
                double tau_grav = tau_grav_vec(static_cast<Eigen::Index>(j));

                double tau_ng = tau_k - tau_grav;

                Y_vec.push_back(tau_ng);
                W1_vec.push_back((dq_k >= 0) ? 1.0 : -1.0);  // sign(dq)
                W2_vec.push_back(dq_k);
            }
        }

        Eigen::Index n = static_cast<Eigen::Index>(Y_vec.size());
        std::cout << "j" << j << ": " << n << " 个数据点\n";

        if (n < 4) {
            std::cerr << "  数据点不足，跳过\n";
            fc_results[j] = fv_results[j] = rmse_results[j] = 0.0;
            continue;
        }

        Eigen::MatrixXd W(n, 2);
        Eigen::VectorXd Y(n);
        for (Eigen::Index i = 0; i < n; ++i) {
            W(i, 0) = W1_vec[static_cast<std::size_t>(i)];
            W(i, 1) = W2_vec[static_cast<std::size_t>(i)];
            Y(i)    = Y_vec[static_cast<std::size_t>(i)];
        }

        // 普通最小二乘（无正则化）
        Eigen::VectorXd beta_f = W.colPivHouseholderQr().solve(Y);
        Eigen::VectorXd residual = W * beta_f - Y;
        double rmse = std::sqrt(residual.squaredNorm() / static_cast<double>(n));

        fc_results[j] = beta_f(0);
        fv_results[j] = beta_f(1);
        rmse_results[j] = rmse;
    }

    // ---- 输出结果 -----------------------------------------------------------
    std::cout << "\n";
    printSep();
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "关节      Fc (Nm)        Fv (Nm·s/rad)   残差 RMSE   数据点数\n";
    printSep();
    for (std::size_t j = 0; j < dof; ++j) {
        std::cout << "j" << j << "        "
                  << std::setw(14) << fc_results[j] << "  "
                  << std::setw(14) << fv_results[j] << "  "
                  << std::setw(14) << rmse_results[j] << "\n";
    }
    printSep();

    for (std::size_t j = 0; j < dof; ++j) {
        if (fc_results[j] < 0.0)
            std::cout << "⚠ j" << j << " Fc < 0 (" << fc_results[j] << ")\n";
        if (fv_results[j] < 0.0)
            std::cout << "⚠ j" << j << " Fv < 0 (" << fv_results[j] << ")\n";
    }

    // ---- 保存 YAML ----------------------------------------------------------
    std::string output_yaml = robot_utils::resultFrictionPath(robot_dir,
        robot_name + "_friction_identification.yaml");
    std::filesystem::create_directories(
        robot_utils::resultFrictionPath(robot_dir, ""));

    std::ofstream out(output_yaml);
    if (out) {
        std::time_t now = std::time(nullptr);
        char buf[64]; std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
                                    std::localtime(&now));
        out << std::fixed << std::setprecision(3);
        out << "calibration_date: \"" << buf << "\"\n"
            << "robot: \"" << robot_name << "\"\n"
            << "method: \"ols_gravity_exact_removal\"\n"
            << "interval_shrink: " << interval_shrink << "\n"
            << "parameters:\n"
            << "  [\n";
        for (std::size_t j = 0; j < dof; ++j) {
            // physical feasible: clamp negative to 0 in saved file
            double fc_save = (fc_results[j] < 0.0) ? 0.0 : fc_results[j];
            double fv_save = (fv_results[j] < 0.0) ? 0.0 : fv_results[j];
            out << "    " << fc_save << ", " << fv_save;
            if (j + 1 < dof) out << ",";
            out << "\n";
        }
        out << "  ]\n";
    }

    std::cout << "\n结果已保存: " << output_yaml << "\n";
    return 0;
}
