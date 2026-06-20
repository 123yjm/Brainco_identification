#include "dynamics_params.hpp"

namespace robot_dynamics {

// ============================================================================
// InertialParams::fromBody
// ============================================================================

InertialParams
InertialParams::fromBody(const RigidBody &body) {
  InertialParams params;

  params.m = body.mass;

  // 一阶矩: m * c
  params.mx = body.mass * body.com.x();
  params.my = body.mass * body.com.y();
  params.mz = body.mass * body.com.z();

  // 惯量张量从质心转换到 Body 原点 (平行轴定理)
  // I_origin = I_com + m * (c^T * c * I - c * c^T)
  double cx = body.com.x();
  double cy = body.com.y();
  double cz = body.com.z();
  double m = body.mass;

  params.Ixx = body.Ixx + m * (cy * cy + cz * cz);
  params.Iyy = body.Iyy + m * (cx * cx + cz * cz);
  params.Izz = body.Izz + m * (cx * cx + cy * cy);
  params.Ixy = body.Ixy - m * cx * cy;
  params.Ixz = body.Ixz - m * cx * cz;
  params.Iyz = body.Iyz - m * cy * cz;

  return params;
}

} // namespace robot_dynamics
