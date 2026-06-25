/**
 * @file main_get_friction_traj.cpp
 * @brief 摩擦力辨识轨迹生成 — 各关节分时匀速运动
 *
 * 用法: ./get_friction_traj --robot <robot_dir>
 *
 * 对每个关节依次执行 4 段匀速运动：+v1, -v1, +v2, -v2，
 * 非运动关节锁定在 q_init。输出 CSV 与激励轨迹格式完全一致。
 */

#include "robot_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 梯形速度 profile 的单段结果
// ---------------------------------------------------------------------------
struct SegmentProfile {
    std::vector<double> time;   // 相对于段起点的时间
    std::vector<double> q;      // 位置
    std::vector<double> qd;     // 速度
    std::vector<double> qdd;    // 加速度
    double duration = 0.0;      // 段总时长
};

// ---------------------------------------------------------------------------
// 帮助信息
// ---------------------------------------------------------------------------
void printHelp(const char* prog) {
    std::cout
        << "摩擦力辨识轨迹生成\n\n"
        << "用法: " << prog << " --robot <robot_dir>\n\n"
        << "选项:\n"
        << "  --robot <dir>  机器人目录 (如 robots/revoarm_right)\n"
        << "  --help         打印帮助信息\n\n"
        << "配置文件: <robot_dir>/config/friction_trajectory.yaml\n"
        << "输出:     <robot_dir>/result/<robot>_friction_trajectory.csv\n";
}

// ---------------------------------------------------------------------------
// 生成单段梯形（或三角形）速度 profile
//
// q_start → q_end, 目标速度 magnitude = v (正数), 加速度 magnitude = a
// sign 由 (q_end - q_start) 自动确定。
// dt 为采样间隔。
// ---------------------------------------------------------------------------
SegmentProfile makeSegment(double q_start, double q_end, double v, double a, double dt) {
    SegmentProfile seg;
    double distance = std::abs(q_end - q_start);
    int sign = (q_end >= q_start) ? 1 : -1;

    // 计算梯形参数
    double t_acc, t_const, t_total;
    double d_acc_needed = v * v / (2.0 * a);  // 加速到 v 所需的距离

    if (distance < 1e-12) {
        // 零行程：单点
        seg.time  = {0.0};
        seg.q     = {q_start};
        seg.qd    = {0.0};
        seg.qdd   = {0.0};
        seg.duration = 0.0;
        return seg;
    }

    if (2.0 * d_acc_needed <= distance) {
        // 梯形 profile
        t_acc   = v / a;
        t_const = (distance - 2.0 * d_acc_needed) / v;
        t_total = 2.0 * t_acc + t_const;
    } else {
        // 三角形 profile（行程太短，达不到满速）
        t_acc   = std::sqrt(distance / a);
        t_const = 0.0;
        v       = a * t_acc;  // 实际达到的最大速度
        t_total = 2.0 * t_acc;
    }

    // 采样
    int n_samples = static_cast<int>(std::ceil(t_total / dt)) + 1;
    seg.time.resize(n_samples);
    seg.q.resize(n_samples);
    seg.qd.resize(n_samples);
    seg.qdd.resize(n_samples);

    for (int i = 0; i < n_samples; ++i) {
        double t = i * dt;
        if (t > t_total) t = t_total;

        double pos, vel, acc;
        if (t <= t_acc) {
            // 加速段
            double tau = t;
            pos = q_start + 0.5 * a * tau * tau * sign;
            vel = a * tau * sign;
            acc = a * sign;
        } else if (t <= t_acc + t_const) {
            // 匀速段
            double tau = t - t_acc;
            double q_at_acc_end = q_start + 0.5 * a * t_acc * t_acc * sign;
            pos = q_at_acc_end + v * tau * sign;
            vel = v * sign;
            acc = 0.0;
        } else {
            // 减速段
            double tau = t - t_acc - t_const;
            double q_at_const_end = q_start
                + 0.5 * a * t_acc * t_acc * sign
                + v * t_const * sign;
            pos = q_at_const_end
                + v * tau * sign
                - 0.5 * a * tau * tau * sign;
            vel = v * sign - a * tau * sign;
            acc = -a * sign;
        }

        seg.time[i]  = t;
        seg.q[i]     = pos;
        seg.qd[i]    = vel;
        seg.qdd[i]   = acc;
    }

    seg.duration = t_total;
    return seg;
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

    std::string robot_name     = robot_utils::robotNameFromDir(robot_dir);
    std::string friction_yaml  = robot_utils::configPath(robot_dir, "friction_trajectory.yaml");
    std::string kinematic_yaml = robot_utils::configPath(robot_dir, "kinematic_params.yaml");
    std::string output_csv     = robot_utils::resultFrictionPath(robot_dir,
        robot_name + "_friction_trajectory.csv");

    // ---- 加载 kinematic_params.yaml（仅取 dof）-------------------------------
    std::size_t dof = 0;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["dof"]) dof = kroot["dof"].as<std::size_t>();
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法读取 kinematic_params.yaml — " << e.what() << std::endl;
        return 1;
    }
    if (dof == 0) {
        std::cerr << "错误: kinematic_params.yaml 缺少 dof 字段\n";
        return 1;
    }

    // ---- 加载 friction_trajectory.yaml -------------------------------------
    double sampling_freq, acceleration, v1, v2;
    Eigen::VectorXd q_init, q_target_v1, q_target_v2;

    try {
        auto root = YAML::LoadFile(friction_yaml);
        sampling_freq = root["sampling_frequency"].as<double>();
        acceleration  = root["acceleration"].as<double>();
        v1            = root["v1"].as<double>();
        v2            = root["v2"].as<double>();
        q_init        = parseVector(root["q_init"]);
        q_target_v1   = parseVector(root["q_target_v1"]);
        q_target_v2   = parseVector(root["q_target_v2"]);

        if (static_cast<std::size_t>(q_init.size()) != dof ||
            static_cast<std::size_t>(q_target_v1.size()) != dof ||
            static_cast<std::size_t>(q_target_v2.size()) != dof) {
            std::cerr << "错误: q_init / q_target_v1 / q_target_v2 维度必须等于 dof=" << dof << std::endl;
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法读取 friction_trajectory.yaml — " << e.what() << std::endl;
        return 1;
    }

    double dt = 1.0 / sampling_freq;

    std::cout << "═══════════════════════════════════════════\n"
              << "  摩擦力辨识轨迹生成\n"
              << "═══════════════════════════════════════════\n"
              << "机器人:       " << robot_name << "\n"
              << "DOF:          " << dof << "\n"
              << "配置:         " << friction_yaml << "\n"
              << "输出:         " << output_csv << "\n"
              << "采样频率:     " << sampling_freq << " Hz\n"
              << "加速度:       " << acceleration << " rad/s^2\n"
              << "低速 v1:      " << v1 << " rad/s\n"
              << "高速 v2:      " << v2 << " rad/s\n";

    // ---- 逐关节生成轨迹 ----------------------------------------------------
    // 总轨迹：各段时间轴拼接
    std::vector<double>      time_all;
    std::vector<Eigen::VectorXd> q_all;    // 每个采样点的 7 维位置
    std::vector<Eigen::VectorXd> qd_all;
    std::vector<Eigen::VectorXd> qdd_all;

    double t_offset = 0.0;

    for (std::size_t j = 0; j < dof; ++j) {
        double q_now = q_init(j);

        const int n_seg = 4;
        double targets[4] = { q_target_v1(j), q_init(j), q_target_v2(j), q_init(j) };
        double speeds[4]  = { v1, v1, v2, v2 };
        const char* labels[4] = { "+v1", "-v1", "+v2", "-v2" };

        std::cout << "\n关节 " << j << " (q_init=" << q_init(j)
                  << ", q_target_v1=" << q_target_v1(j)
                  << ", q_target_v2=" << q_target_v2(j) << "):\n";

        for (int s = 0; s < n_seg; ++s) {
            double q_goal = targets[s];
            double speed  = speeds[s];

            SegmentProfile prof = makeSegment(q_now, q_goal, speed, acceleration, dt);

            std::cout << "  " << labels[s] << ": " << q_now << " -> " << q_goal
                      << ", 行程=" << std::abs(q_goal - q_now)
                      << ", 时长=" << prof.duration << "s"
                      << ", 采样点=" << prof.time.size() << std::endl;

            // 将段内采样点追加到总轨迹
            for (std::size_t k = 0; k < prof.time.size(); ++k) {
                double t_abs = t_offset + prof.time[k];

                Eigen::VectorXd qi(dof), qdi(dof), qddi(dof);
                // 非运动关节：锁死在 q_init
                for (std::size_t jj = 0; jj < dof; ++jj) {
                    if (jj == j) {
                        qi(jj)   = prof.q[k];
                        qdi(jj)  = prof.qd[k];
                        qddi(jj) = prof.qdd[k];
                    } else {
                        qi(jj)   = q_init(jj);
                        qdi(jj)  = 0.0;
                        qddi(jj) = 0.0;
                    }
                }

                time_all.push_back(t_abs);
                q_all.push_back(qi);
                qd_all.push_back(qdi);
                qdd_all.push_back(qddi);
            }

            t_offset += prof.duration;
            q_now = q_goal;
        }
    }

    // ---- 写入 CSV -----------------------------------------------------------
    std::ofstream out(output_csv);
    if (!out) {
        std::cerr << "错误: 无法写入 " << output_csv << std::endl;
        return 1;
    }

    out << "time";
    for (std::size_t j = 0; j < dof; ++j) out << ",q" << j;
    for (std::size_t j = 0; j < dof; ++j) out << ",qd" << j;
    for (std::size_t j = 0; j < dof; ++j) out << ",qdd" << j;
    for (std::size_t j = 0; j < dof; ++j) out << ",tau" << j;
    out << "\n";

    out << std::scientific << std::setprecision(15);
    std::size_t K = time_all.size();
    for (std::size_t k = 0; k < K; ++k) {
        out << time_all[k];
        for (std::size_t j = 0; j < dof; ++j) out << "," << q_all[k](static_cast<Eigen::Index>(j));
        for (std::size_t j = 0; j < dof; ++j) out << "," << qd_all[k](static_cast<Eigen::Index>(j));
        for (std::size_t j = 0; j < dof; ++j) out << "," << qdd_all[k](static_cast<Eigen::Index>(j));
        for (std::size_t j = 0; j < dof; ++j) out << "," << 0.0;
        out << "\n";
    }

    std::cout << "\n完成: " << output_csv << " (" << K << " 行, "
              << t_offset << "s)\n";
    return 0;
}
