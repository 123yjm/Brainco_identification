#pragma once

#include "rigid_body.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace robot_dynamics {

/// 从 kinematic_params.yaml 解析出的机器人配置
struct RobotConfig {
  std::string robot_name;
  std::size_t dof = 0;
  std::size_t kinematic_prefix = 0;
  std::vector<RigidBody> bodies;
};

/// 从 YAML 文件加载机器人运动学/动力学配置
/// 必填字段: robot_name, dof, bodies (每个 body 至少需要 name 和 has_joint)
/// 可选字段: pos, quat, mass, com, Ixx-Iyz, joint_axis（自动取默认值）
/// joint_axis 在 has_joint=true 时必填
RobotConfig loadKinematicConfig(const std::string &yaml_path);

} // namespace robot_dynamics
