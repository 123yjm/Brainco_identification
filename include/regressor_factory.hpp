#pragma once

#include "dynamics_regressor.hpp"

#include <memory>
#include <string>

namespace robot_dynamics {

/// 根据 robot 名称创建对应的 regressor 实例
struct RegressorFactory {
  /// @param robot_name   identification.yaml 中的 robot 字段
  /// @param yaml_path    运动学参数 YAML 文件路径
  /// @return             对应的 IDynamicsRegressor 实例
  /// @throws std::runtime_error   robot_name 未识别
  static std::unique_ptr<IDynamicsRegressor> create(
      const std::string &robot_name,
      const std::string &yaml_path);
};

} // namespace robot_dynamics
