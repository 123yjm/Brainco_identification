/**
 * @file main_solve_friction.cpp
 * @brief 摩擦力辨识 — Coulomb + Viscous 摩擦系数求解
 *
 * 用法: ./identify_friction --robot <robot_dir>
 *
 * 基于分时匀速运动数据，通过正反向速度配对消除重力项，
 * 对每个关节独立求解 Fc (库伦摩擦) 和 Fv (粘性摩擦)。
 *
 * 摩擦模型: tau_friction = Fc * sign(dq) + Fv * dq
 */

#include "data_loader.hpp"
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
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 帮助信息
// ---------------------------------------------------------------------------
void printHelp(const char* prog) {
    std::cout
        << "摩擦力辨识 — Coulomb + Viscous\n\n"
        << "用法: " << prog << " --robot <robot_dir>\n\n"
        << "选项:\n"
        << "  --robot <dir>  机器人目录 (如 robots/revoarm_right)\n"
        << "  --help         打印帮助信息\n\n"
        << "读取 data_friction/*.csv，对每个关节独立求解 Fc, Fv。\n";
}

// ---------------------------------------------------------------------------
// 解析 YAML 浮点数向量
// ---------------------------------------------------------------------------
Eigen::VectorXd parseVector(const YAML::Node& node) {
    Eigen::VectorXd v(node.size());
    for (std::size_t i = 0; i < node.size(); ++i)
        v(static_cast<Eigen::Index>(i)) = node[i].as<double>();
    return v;
}

// ---------------------------------------------------------------------------
// 分离线
// ---------------------------------------------------------------------------
void printSep() { std::cout << std::string(55, '-') << "\n"; }

}  // anonymous namespace

// ============================================================================
// main
// ============================================================================
int main(int argc, char* argv[]) {
    // ---- 解析 CLI ----------------------------------------------------------
    std::string robot_dir;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") { printHelp(argv[0]); return 0; }
        else if ((arg == "--robot" || arg == "-r") && i + 1 < argc)
            robot_dir = robot_utils::resolvePath(
                robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
    }
    if (robot_dir.empty()) {
        std::cerr << "错误: 需要 --robot <dir>\n使用 --help 查看用法\n";
        return 1;
    }

    std::string robot_name        = robot_utils::robotNameFromDir(robot_dir);
    std::string friction_yaml     = robot_utils::configPath(robot_dir, "friction_trajectory.yaml");
    std::string friction_id_yaml  = robot_utils::configPath(robot_dir, "friction_identification.yaml");
    std::string kinematic_yaml    = robot_utils::configPath(robot_dir, "kinematic_params.yaml");

    // ---- 加载 kinematic_params.yaml（取 dof）---------------------------------
    std::size_t dof = 0;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["dof"]) dof = kroot["dof"].as<std::size_t>();
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法读取 kinematic_params.yaml — " << e.what() << std::endl;
        return 1;
    }

    // ---- 加载 friction_trajectory.yaml -------------------------------------
    double a, v1, v2;
    Eigen::VectorXd q_init, q_target;

    try {
        auto root = YAML::LoadFile(friction_yaml);
        a       = root["acceleration"].as<double>();
        v1      = root["v1"].as<double>();
        v2      = root["v2"].as<double>();
        q_init  = parseVector(root["q_init"]);
        q_target = parseVector(root["q_target"]);
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法读取 friction_trajectory.yaml — " << e.what() << std::endl;
        return 1;
    }

    // ---- 加载 friction_identification.yaml ---------------------------------
    double interval_shrink = 1.0;
    try {
        auto froot = YAML::LoadFile(friction_id_yaml);
        if (froot["interval_shrink"])
            interval_shrink = froot["interval_shrink"].as<double>();
    } catch (...) {}

    // ---- 查找摩擦数据 CSV ---------------------------------------------------
    std::string data_file = robot_utils::findFirstFile(
        robot_utils::dataFrictionPath(robot_dir, ""), "*.csv");
    if (data_file.empty()) {
        std::cerr << "错误: 在 " << robot_utils::dataFrictionPath(robot_dir, "")
                  << " 下未找到 .csv 文件\n";
        return 1;
    }

    // ---- 加载数据 -----------------------------------------------------------
    ExperimentData data;
    try {
        data = DataLoader::loadCSV(data_file, dof);
    } catch (const std::exception& e) {
        std::cerr << "错误: 数据加载失败 — " << e.what() << std::endl;
        return 1;
    }

    // ---- 计算各关节匀速段时间窗口（含 interval_shrink）----------------------
    struct Window {
        double t_start, t_end;
        double speed;   // signed velocity
        int joint;
        std::string label;
    };

    std::vector<Window> windows;
    double t_global = 0.0;

    for (std::size_t j = 0; j < dof; ++j) {
        double q_now = q_init(j);
        const int n_seg = 4;
        double targets[4] = { q_target(j), q_init(j), q_target(j), q_init(j) };
        double speeds[4]  = { v1, v1, v2, v2 };
        const char* labels[4] = { "+v1", "-v1", "+v2", "-v2" };

        for (int s = 0; s < n_seg; ++s) {
            double q_goal = targets[s];
            double v      = speeds[s];
            double distance = std::abs(q_goal - q_now);

            double t_const = 0.0, t_acc = 0.0;
            if (distance > 1e-12) {
                double d_acc = v * v / (2.0 * a);
                t_acc = v / a;
                if (2.0 * d_acc <= distance) {
                    t_const = (distance - 2.0 * d_acc) / v;
                } else {
                    t_acc = std::sqrt(distance / a);
                    t_const = 0.0;
                }
            }

            double t_total = 2.0 * t_acc + t_const;

            if (t_const > 0.0) {
                double t_start = t_global + t_acc;
                double t_end   = t_global + t_acc + t_const;

                // 向中心缩放窗口
                double center = (t_start + t_end) / 2.0;
                double half   = (t_end - t_start) / 2.0 * interval_shrink;
                double t_start_s = center - half;
                double t_end_s   = center + half;

                // 确定实际速度符号
                double sign = (q_goal >= q_now) ? 1.0 : -1.0;

                windows.push_back({t_start_s, t_end_s, sign * v,
                                   static_cast<int>(j), labels[s]});
            }

            t_global += t_total;
            q_now = q_goal;
        }
    }

    // ---- 打印配置 -----------------------------------------------------------
    std::cout << "═══════════════════════════════════════════\n"
              << "  摩擦力辨识 — Coulomb + Viscous\n"
              << "═══════════════════════════════════════════\n"
              << "机器人:           " << robot_name << "\n"
              << "数据文件:         " << data_file << "\n"
              << "样本数:           " << data.n_samples << "\n"
              << "v1 / v2:          " << v1 << " / " << v2 << " rad/s\n"
              << "interval_shrink:  " << interval_shrink << "\n\n";

    // ---- 逐关节求解 Fc, Fv ------------------------------------------------
    std::vector<double> fc_results(dof), fv_results(dof), rmse_results(dof);

    for (std::size_t j = 0; j < dof; ++j) {
        // 收集该关节的 4 个窗口
        std::vector<Window> jw;
        for (auto& w : windows)
            if (w.joint == static_cast<int>(j)) jw.push_back(w);

        // 按速度大小分组：+v1 配 -v1, +v2 配 -v2
        // +v1 和 -v1 的 speed magnitude 都是 v1
        std::vector<double> Y_vals, V_vals;

        // 配对 v1: 找 +v1 和 -v1 (speed ≈ +v1 和 -v1)
        for (double v : {v1, v2}) {
            Window *wp = nullptr, *wn = nullptr;
            for (auto& w : jw) {
                if (std::abs(std::abs(w.speed) - v) < 1e-9) {
                    if (w.speed > 0) wp = &w;
                    else             wn = &w;
                }
            }
            if (!wp || !wn) continue;

            // 提取两个窗口的数据
            auto extract = [&](const Window& w, std::vector<double>& tau_out) {
                tau_out.clear();
                for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(data.n_samples); ++k) {
                    if (data.time[k] >= w.t_start && data.time[k] <= w.t_end) {
                        tau_out.push_back(data.tau(k, static_cast<Eigen::Index>(j)));
                    }
                }
            };

            std::vector<double> tau_plus, tau_minus;
            extract(*wp, tau_plus);
            extract(*wn, tau_minus);

            if (tau_plus.empty() || tau_minus.empty()) continue;

            // 计算 mean(tau_plus) 和 mean(tau_minus)，然后消除重力
            double mean_plus  = 0.0, mean_minus = 0.0;
            for (double x : tau_plus)  mean_plus  += x;
            for (double x : tau_minus) mean_minus += x;
            mean_plus  /= tau_plus.size();
            mean_minus /= tau_minus.size();

            double y = (mean_plus - mean_minus) / 2.0;  // 重力消除

            Y_vals.push_back(y);
            V_vals.push_back(v);
        }

        // 构建 W (n x 2) 和 Y (n x 1)
        Eigen::Index n = static_cast<Eigen::Index>(Y_vals.size());
        if (n < 2) {
            std::cerr << "警告: 关节 " << j << " 有效速度挡位不足，跳过\n";
            fc_results[j] = fv_results[j] = rmse_results[j] = 0.0;
            continue;
        }

        Eigen::MatrixXd W(n, 2);
        Eigen::VectorXd Y(n);
        for (Eigen::Index i = 0; i < n; ++i) {
            W(i, 0) = 1.0;
            W(i, 1) = V_vals[static_cast<std::size_t>(i)];
            Y(i)    = Y_vals[static_cast<std::size_t>(i)];
        }

        // 普通最小二乘（无正则化）
        Eigen::VectorXd beta = W.colPivHouseholderQr().solve(Y);

        double Fc = beta(0);
        double Fv = beta(1);

        // 残差 RMSE
        Eigen::VectorXd residual = W * beta - Y;
        double rmse = std::sqrt(residual.squaredNorm() / static_cast<double>(n));

        fc_results[j]   = Fc;
        fv_results[j]   = Fv;
        rmse_results[j] = rmse;
    }

    // ---- 输出结果 -----------------------------------------------------------
    printSep();
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "关节      Fc (Nm)        Fv (Nm·s/rad)   残差 RMSE\n";
    printSep();
    for (std::size_t j = 0; j < dof; ++j) {
        std::cout << "j" << j << "        "
                  << std::setw(14) << fc_results[j] << "  "
                  << std::setw(14) << fv_results[j] << "  "
                  << std::setw(14) << rmse_results[j] << "\n";
    }
    printSep();

    // 物理有效性检查
    for (std::size_t j = 0; j < dof; ++j) {
        if (fc_results[j] < 0.0)
            std::cout << "⚠ j" << j << " Fc < 0 (" << fc_results[j] << ")，可能异常\n";
        if (fv_results[j] < 0.0)
            std::cout << "⚠ j" << j << " Fv < 0 (" << fv_results[j] << ")，可能异常\n";
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
        out << std::scientific << std::setprecision(15);
        out << "calibration_date: \"" << buf << "\"\n"
            << "robot: \"" << robot_name << "\"\n"
            << "method: \"ols_gravity_cancellation\"\n"
            << "interval_shrink: " << interval_shrink << "\n"
            << "v1: " << v1 << "\n"
            << "v2: " << v2 << "\n"
            << "joints:\n";
        for (std::size_t j = 0; j < dof; ++j) {
            out << "  - joint: " << j << "\n"
                << "    Fc: " << fc_results[j] << "\n"
                << "    Fv: " << fv_results[j] << "\n"
                << "    rmse: " << rmse_results[j] << "\n";
        }
    }

    std::cout << "\n结果已保存: " << output_yaml << "\n";
    return 0;
}
