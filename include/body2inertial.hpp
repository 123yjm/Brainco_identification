#pragma once

#include "rigid_body.hpp"

#include <Eigen/Core>
#include <array>
#include <cstddef>

namespace robot_dynamics {

// ============================================================================
// ParamFlags — 额外动力学参数类型标志
// ============================================================================

enum class ParamFlags : unsigned int {
  NONE = 0,
  DAMPING = 1 << 0,   ///< 粘性阻尼 (符号为负)
  ARMATURE = 1 << 1,  ///< 电机转子反映惯量
  ALL = DAMPING | ARMATURE
};

inline ParamFlags operator|(ParamFlags a, ParamFlags b) {
  return static_cast<ParamFlags>(static_cast<unsigned>(a) |
                                       static_cast<unsigned>(b));
}

inline ParamFlags operator&(ParamFlags a, ParamFlags b) {
  return static_cast<ParamFlags>(static_cast<unsigned>(a) &
                                       static_cast<unsigned>(b));
}

inline bool hasFlag(ParamFlags flags, ParamFlags test) {
  return (static_cast<unsigned>(flags) & static_cast<unsigned>(test)) != 0;
}

// ============================================================================
// InertialParams — 标准惯性参数 (每个 Body 10 个)
// ============================================================================

struct InertialParams {
  static constexpr std::size_t PARAMS_PER_BODY = 10;

  double m;   ///< 质量
  double mx;  ///< 一阶矩 x
  double my;  ///< 一阶矩 y
  double mz;  ///< 一阶矩 z
  double Ixx; ///< 惯量张量 xx (Body 原点)
  double Ixy; ///< 惯量张量 xy
  double Ixz; ///< 惯量张量 xz
  double Iyy; ///< 惯量张量 yy
  double Iyz; ///< 惯量张量 yz
  double Izz; ///< 惯量张量 zz

  Eigen::Matrix<double, 10, 1> toVector() const {
    Eigen::Matrix<double, 10, 1> v;
    v << m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
    return v;
  }

  /// 从 RigidBody 创建标准惯性参数（含平行轴定理变换: COM → Body 原点）
  static InertialParams fromBody(const RigidBody &body);
};

} // namespace robot_dynamics
