/**
 * @file main_remove_gravity.cpp
 * @brief 重力分离 — 从采集力矩中扣除重力项
 *
 * 用法: ./remove_gravity --robot <robot_dir> [--data <csv>]
 *
 * tau_without_gravity = tau - Y(q, 0, 0) * beta
 * 输出: result_others/<robot>_gravity_removed.csv
 */

#include "regressor_factory.hpp"
#include "robot_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
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
        << "重力分离 — 从采集力矩中扣除重力项\n\n"
        << "用法: " << prog << " --robot <robot_dir> [--data <csv_path>]\n\n"
        << "选项:\n"
        << "  --robot <dir>    机器人目录 (如 robots/revoarm_right)\n"
        << "  --data <csv>     轨迹数据 CSV（默认自动找 data_friction/*.csv）\n"
        << "  --help           打印帮助信息\n\n"
        << "beta 参数自动从 result_inertia/<robot>_inertia_identification.yaml 加载。\n";
}

}  // anonymous namespace

int main(int argc, char* argv[]) {
    // ---- 解析 CLI ----------------------------------------------------------
    std::string robot_dir, data_file;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") { printHelp(argv[0]); return 0; }
        else if ((arg == "--robot" || arg == "-r") && i + 1 < argc)
            robot_dir = robot_utils::resolvePath(
                robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
        else if (arg == "--data" && i + 1 < argc)
            data_file = argv[++i];
    }
    if (robot_dir.empty()) {
        std::cerr << "错误: 需要 --robot <dir>\n"; return 1;
    }

    std::string robot_name     = robot_utils::robotNameFromDir(robot_dir);
    std::string kinematic_yaml = robot_utils::configPath(robot_dir, "kinematic_params.yaml");
    std::string beta_yaml      = robot_utils::resultInertiaPath(robot_dir,
        robot_name + "_inertia_identification.yaml");

    // auto-detect data CSV
    if (data_file.empty()) {
        data_file = robot_utils::findFirstFile(
            robot_utils::dataFrictionPath(robot_dir, ""), "*.csv");
        if (data_file.empty()) {
            data_file = robot_utils::findFirstFile(
                robot_utils::dataInertiaPath(robot_dir, ""), "*.csv");
        }
    }
    if (data_file.empty()) {
        std::cerr << "错误: 未找到数据 CSV\n"; return 1;
    }

    // ---- 读取 robot_type ---------------------------------------------------
    std::string robot_type = robot_name;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["robot_type"]) robot_type = kroot["robot_type"].as<std::string>();
    } catch (...) {}

    // ---- 创建 regressor ----------------------------------------------------
    std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor;
    try {
        regressor = robot_dynamics::RegressorFactory::create(robot_type, kinematic_yaml);
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法创建 regressor — " << e.what() << "\n"; return 1;
    }

    const std::size_t dof = regressor->nDof();

    // ---- 加载 beta ---------------------------------------------------------
    Eigen::VectorXd beta;
    try {
        auto root = YAML::LoadFile(beta_yaml);
        auto params_node = root["benchmark_results"][0]["parameters"];
        // params_node is a YAML flow-style sequence; flatten it
        std::vector<double> vals;
        for (const auto& item : params_node) {
            // Each item is a sub-sequence of 10 doubles (per line)
            if (item.IsSequence()) {
                for (const auto& v : item)
                    vals.push_back(v.as<double>());
            } else {
                // standalone double (last line may have fewer elements)
                vals.push_back(item.as<double>());
            }
        }
        beta = Eigen::VectorXd(vals.size());
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(vals.size()); ++i)
            beta(i) = vals[i];
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法加载 beta — " << e.what() << "\n"; return 1;
    }

    // ---- 逐帧处理 CSV ------------------------------------------------------
    std::string output_csv = robot_utils::resultOthersPath(robot_dir,
        robot_name + "_gravity_removed.csv");
    std::filesystem::create_directories(
        robot_utils::resultOthersPath(robot_dir, ""));

    std::ifstream in(data_file);
    if (!in) { std::cerr << "错误: 无法打开 " << data_file << "\n"; return 1; }
    std::ofstream out(output_csv);
    if (!out) { std::cerr << "错误: 无法写入 " << output_csv << "\n"; return 1; }
    out << std::scientific << std::setprecision(15);

    std::string line;
    std::getline(in, line);  // header — write unchanged
    out << line << "\n";

    const std::size_t ncols = 1 + 6 * dof;  // 43 for 7-DOF

    Eigen::VectorXd q(dof), qd(dof), tau(dof);
    Eigen::VectorXd zeros = Eigen::VectorXd::Zero(dof);
    auto flags = robot_dynamics::ParamFlags::ALL;

    std::size_t frame_count = 0;

    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> row;
        while (std::getline(ss, token, ','))
            row.push_back(std::stod(token));

        if (row.size() < ncols) continue;  // skip malformed rows

        // Extract q_actual (cols 1..1+dof) and tau_actual (cols 1+2*dof..1+3*dof)
        for (std::size_t j = 0; j < dof; ++j) {
            q(j)   = row[1 + j];
            qd(j)  = row[1 + dof + j];
            tau(j) = row[1 + 2 * dof + j];
        }

        // Compute gravity: Y(q, 0, 0) * beta
        Eigen::MatrixXd Y_grav = regressor->computeRegressorMatrix(q, zeros, zeros, flags);
        Eigen::VectorXd tau_grav = Y_grav * beta;
        Eigen::VectorXd tau_new = tau - tau_grav;

        // Write row with modified tau_actual
        out << row[0];  // time
        for (std::size_t j = 0; j < dof; ++j) out << "," << row[1 + j];           // q_actual
        for (std::size_t j = 0; j < dof; ++j) out << "," << row[1 + dof + j];     // dq_actual
        for (std::size_t j = 0; j < dof; ++j) out << "," << tau_new(j);           // tau_actual (modified)
        for (std::size_t j = 0; j < dof; ++j) out << "," << row[1 + 3 * dof + j]; // q_ref
        for (std::size_t j = 0; j < dof; ++j) out << "," << row[1 + 4 * dof + j]; // dq_ref
        for (std::size_t j = 0; j < dof; ++j) out << "," << row[1 + 5 * dof + j]; // tau_ref
        out << "\n";

        frame_count++;
    }

    std::cout << "═══════════════════════════════════════════\n"
              << "  重力分离\n"
              << "═══════════════════════════════════════════\n"
              << "机器人:   " << robot_name << "\n"
              << "数据:     " << data_file << "\n"
              << "beta:     " << beta_yaml << "\n"
              << "帧数:     " << frame_count << "\n"
              << "DOF:      " << dof << "\n"
              << "输出:     " << output_csv << "\n"
              << "完成\n";

    return 0;
}
