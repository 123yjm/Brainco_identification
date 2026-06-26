#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

/**
 * @brief 纯数据结构: Rigid Body 的惯性/运动学/关节参数
 *
 * 惯量 Ixx..Izz 定义在 Body 的质心坐标系。
 * 使用 fromBody() 转换为回归器所需的原点坐标系参数。
 */
struct RigidBody {
  using Vector3d = Eigen::Vector3d;
  using Quaterniond = Eigen::Quaterniond;

  std::string name;
  Vector3d pos{0, 0, 0};
  Quaterniond quat{1, 0, 0, 0};
  double mass = 0;
  Vector3d com{0, 0, 0};
  double Ixx = 0, Iyy = 0, Izz = 0;
  double Ixy = 0, Ixz = 0, Iyz = 0;
  Vector3d joint_axis{0, 0, 1};
  bool has_joint = false;

  /// 标志位: 对应先验字段是否在 YAML 中显式设定
  bool has_mass = false;
  bool has_com_x = false, has_com_y = false, has_com_z = false;
  bool has_Ixx = false, has_Iyy = false, has_Izz = false;
  bool has_Ixy = false, has_Ixz = false, has_Iyz = false;
};
