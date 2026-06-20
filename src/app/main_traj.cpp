/**
 * @file main_traj.cpp
 * @brief 傅里叶激励轨迹优化入口
 *
 * 用法: ./get_traj --robot <robot_dir> [--help]
 */

#include "robot_utils.hpp"
#include "trajectory_optimizer.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void printHelp(const char* prog) {
    std::cout << "傅里叶激励轨迹优化\n"
              << "用法: " << prog << " --robot <robot_dir>\n\n"
              << "选项:\n"
              << "  --robot <dir>  机器人目录 (如 robots/revoarm_right)\n"
              << "  --help         打印帮助信息\n";
}

}  // anonymous namespace

int main(int argc, char* argv[]) {
    std::string robot_dir;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") { printHelp(argv[0]); return 0; }
        else if (arg == "--robot" && i + 1 < argc) robot_dir = argv[++i];
    }

    if (robot_dir.empty()) {
        std::cerr << "错误: 需要 --robot <dir>\n";
        return 1;
    }

    std::string robot_name = robot_utils::robotNameFromDir(robot_dir);
    std::string config_yaml = robot_utils::configPath(robot_dir, "excite_trajectory.yaml");
    std::string kinematic_yaml = robot_utils::configPath(robot_dir, "kinematic_params.yaml");
    std::string output_csv = robot_utils::resultPath(robot_dir,
        robot_name + "_excitation_trajectory.csv");

    // 读取 robot_type
    std::string robot_type = robot_name;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["robot_type"]) robot_type = kroot["robot_type"].as<std::string>();
    } catch (...) {}

    std::cout << "═══════════════════════════════════════════\n"
              << "  激励轨迹优化\n"
              << "═══════════════════════════════════════════\n"
              << "机器人:     " << robot_name << " (type: " << robot_type << ")\n"
              << "配置:       " << config_yaml << "\n"
              << "运动学:     " << kinematic_yaml << "\n"
              << "输出:       " << output_csv << std::endl;

    auto cfg = excitation_trajectory::loadExciteTrajectoryConfig(config_yaml);
    excitation_trajectory::ExcitationTrajectoryOptimizer optimizer(
        cfg, robot_type, kinematic_yaml);

    auto result = optimizer.optimize();
    optimizer.saveTrajectoryCSV(result, output_csv);

    std::cout << "\n完成: " << output_csv << std::endl;
    return 0;
}
