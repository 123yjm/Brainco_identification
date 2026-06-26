#include "robot_config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <stdexcept>

namespace robot_dynamics {

// ---------------------------------------------------------------------------
// 辅助: 从 YAML 序列解析 Vector3d (默认值 {0,0,0})
// ---------------------------------------------------------------------------
static Eigen::Vector3d parseVec3(const YAML::Node &node,
                                  const Eigen::Vector3d &default_val = {0, 0, 0}) {
  if (!node || !node.IsSequence()) return default_val;
  if (node.size() != 3) {
    throw std::runtime_error("Vector3d 需要 3 个元素，实际得到 " +
                             std::to_string(node.size()));
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

// ---------------------------------------------------------------------------
// 辅助: 从 YAML 序列解析 Quaterniond (w,x,y,z 顺序, 默认单位四元数)
// ---------------------------------------------------------------------------
static Eigen::Quaterniond parseQuat(const YAML::Node &node) {
  if (!node || !node.IsSequence()) return {1, 0, 0, 0};
  if (node.size() != 4) {
    throw std::runtime_error("Quaternion 需要 4 个元素 (w,x,y,z)，实际得到 " +
                             std::to_string(node.size()));
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>(),
          node[3].as<double>()};
}

// ---------------------------------------------------------------------------
// 主解析函数
// ---------------------------------------------------------------------------
RobotConfig loadKinematicConfig(const std::string &yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception &e) {
    throw std::runtime_error("无法解析 YAML 文件 " + yaml_path + ": " + e.what());
  }

  RobotConfig cfg;

  // ---- 顶层字段 ----
  // robot_name 为可选（未填则留空，由入口函数根据目录名设置）
  if (root["robot_name"]) {
    cfg.robot_name = root["robot_name"].as<std::string>();
  }

  if (!root["dof"]) {
    throw std::runtime_error("缺少必填字段: dof");
  }
  cfg.dof = root["dof"].as<std::size_t>();

  // kinematic_prefix 可选，默认 0
  if (root["kinematic_prefix"]) {
    cfg.kinematic_prefix = root["kinematic_prefix"].as<std::size_t>();
  }

  // ---- bodies 序列 ----
  if (!root["bodies"] || !root["bodies"].IsSequence()) {
    throw std::runtime_error("缺少必填字段: bodies (必须是序列)");
  }

  for (const auto &body_node : root["bodies"]) {
    RigidBody body;

    // name — 必填
    if (!body_node["name"]) {
      throw std::runtime_error("body 缺少必填字段: name");
    }
    body.name = body_node["name"].as<std::string>();

    // has_joint — 必填
    if (!body_node["has_joint"]) {
      throw std::runtime_error("body \"" + body.name + "\" 缺少必填字段: has_joint");
    }
    body.has_joint = body_node["has_joint"].as<bool>();

    // ---- 可选运动学字段 ----
    body.pos = parseVec3(body_node["pos"]);
    body.quat = parseQuat(body_node["quat"]);

    // ---- 可选动力学字段（物理先验）----
    if (body_node["mass"]) {
      body.mass = body_node["mass"].as<double>();
      body.has_mass = true;
    }

    // COM: 优先解析分轴字段 com_x/com_y/com_z，回退到旧格式 com: [x,y,z]
    bool has_any_com = false;
    if (body_node["com_x"]) {
      body.com.x() = body_node["com_x"].as<double>();
      body.has_com_x = true;
      has_any_com = true;
    }
    if (body_node["com_y"]) {
      body.com.y() = body_node["com_y"].as<double>();
      body.has_com_y = true;
      has_any_com = true;
    }
    if (body_node["com_z"]) {
      body.com.z() = body_node["com_z"].as<double>();
      body.has_com_z = true;
      has_any_com = true;
    }
    // 回退: 旧格式 com: [x, y, z] — 三者全都显式设定
    if (!has_any_com && body_node["com"] && body_node["com"].IsSequence()) {
      body.com = parseVec3(body_node["com"]);
      body.has_com_x = true;
      body.has_com_y = true;
      body.has_com_z = true;
    }

    // 惯量分量各自可选，默认 0
    if (body_node["Ixx"]) { body.Ixx = body_node["Ixx"].as<double>(); body.has_Ixx = true; }
    if (body_node["Iyy"]) { body.Iyy = body_node["Iyy"].as<double>(); body.has_Iyy = true; }
    if (body_node["Izz"]) { body.Izz = body_node["Izz"].as<double>(); body.has_Izz = true; }
    if (body_node["Ixy"]) { body.Ixy = body_node["Ixy"].as<double>(); body.has_Ixy = true; }
    if (body_node["Ixz"]) { body.Ixz = body_node["Ixz"].as<double>(); body.has_Ixz = true; }
    if (body_node["Iyz"]) { body.Iyz = body_node["Iyz"].as<double>(); body.has_Iyz = true; }

    // ---- joint_axis：has_joint=true 时必填 ----
    if (body.has_joint) {
      if (!body_node["joint_axis"]) {
        throw std::runtime_error("body \"" + body.name +
                                 "\" has_joint=true 但缺少 joint_axis");
      }
      body.joint_axis = parseVec3(body_node["joint_axis"]);
    } else if (body_node["joint_axis"]) {
      body.joint_axis = parseVec3(body_node["joint_axis"]);
    }

    cfg.bodies.push_back(body);
  }

  std::cout << "已加载机器人配置: " << cfg.robot_name << ", DOF=" << cfg.dof
            << ", kinematic_prefix=" << cfg.kinematic_prefix
            << ", bodies=" << cfg.bodies.size() << std::endl;

  return cfg;
}

} // namespace robot_dynamics
