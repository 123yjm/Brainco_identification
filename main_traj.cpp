/**
 * @file main_traj.cpp
 * @brief get_traj — 傅里叶激励轨迹优化独立入口
 *
 * 读取 config/excite_trajectory.yaml，优化 5 阶傅里叶级数系数 (a, b, c)。
 * D-最优准则: max det(W^T W)，COBYLA + SLSQP 两阶段求解。
 *
 * 用法:
 *   ./get_traj [--config <excite_trajectory.yaml>]
 */

#include "trajectory_optimizer.hpp"
#include "regressor_factory.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// 项目根目录
// ---------------------------------------------------------------------------
#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

inline std::string resolvePath(const std::string &path) {
  std::filesystem::path p(path);
  if (p.is_absolute()) return path;
  return (std::filesystem::path(PROJECT_ROOT_DIR) / p).string();
}

// ============================================================================
int main(int argc, char *argv[]) {
  std::string config_path = "config/excite_trajectory.yaml";

  // 解析命令行参数
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    }
  }
  config_path = resolvePath(config_path);

  std::cout << "==============================================" << std::endl;
  std::cout << "  激励轨迹优化 — 5 阶傅里叶级数" << std::endl;
  std::cout << "  D-最优准则: max det(W^T W)" << std::endl;
  std::cout << "==============================================" << std::endl;
  std::cout << "配置文件: " << config_path << std::endl;

  // 加载配置
  excitation_trajectory::ExcitationTrajectoryConfig excite_cfg;
  try {
    excite_cfg = excitation_trajectory::loadExciteTrajectoryConfig(config_path);
  } catch (const std::exception &e) {
    std::cerr << "错误: 无法加载配置文件 — " << e.what() << std::endl;
    return 1;
  }

  // 路径解析
  if (!excite_cfg.kinematic_params_path.empty())
    excite_cfg.kinematic_params_path =
        resolvePath(excite_cfg.kinematic_params_path);
  if (!excite_cfg.output_trajectory_path.empty())
    excite_cfg.output_trajectory_path =
        resolvePath(excite_cfg.output_trajectory_path);

  // 运行优化
  excitation_trajectory::ExcitationTrajectoryOptimizer optimizer(excite_cfg);
  auto result = optimizer.optimize();

  // 保存轨迹
  if (!excite_cfg.output_trajectory_path.empty())
    optimizer.saveTrajectoryCSV(result, excite_cfg.output_trajectory_path);

  std::cout << "\n完成。" << std::endl;
  return 0;
}
