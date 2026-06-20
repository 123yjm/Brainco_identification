#pragma once

#include "body2inertial.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <string>
#include <vector>

namespace robot_dynamics {

class RevoarmNewRegressor {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
  using Matrix3d = Eigen::Matrix3d;
  using Vector3d = Eigen::Vector3d;
  using Matrix4d = Eigen::Matrix4d;
  using Quaterniond = Eigen::Quaterniond;

  /// 从 YAML 配置文件构造（唯一入口）
  explicit RevoarmNewRegressor(const std::string &yaml_path);

  /// 自由度数
  std::size_t nDof() const { return n_dof_; }

  /// 用于辨识的刚体数量（不含 kinematic prefix）
  std::size_t nBodies() const { return n_bodies_; }

  VectorXd
  computeParameterVector(ParamFlags flags = ParamFlags::ALL) const;

  std::size_t
  numParameters(ParamFlags flags = ParamFlags::ALL) const;

  MatrixXd
  computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                         const VectorXd &qdd,
                         ParamFlags flags = ParamFlags::ALL) const;

  MatrixXd computeObservationMatrix(
      const MatrixXd &Q, const MatrixXd &Qd, const MatrixXd &Qdd,
      ParamFlags flags = ParamFlags::ALL) const;

  std::vector<std::string>
  getParameterNames(ParamFlags flags = ParamFlags::ALL) const;

private:
  std::size_t n_dof_ = 0;
  std::size_t n_bodies_ = 0;
  std::size_t kinematic_prefix_ = 0;

  /// 全部刚体（从 YAML 加载的原始数据）
  std::vector<RigidBody> bodies_;

  /// Pinocchio 模型与数据
  pinocchio::Model pin_model_;
  mutable pinocchio::Data pin_data_;

  /// 辨识刚体对应的 Pinocchio joint 索引（长度为 n_bodies_）
  std::vector<pinocchio::JointIndex> regressor_joint_ids_;

  Vector3d gravity_{0, 0, -9.81};

  void loadFromYaml(const std::string &path);
  void buildPinocchioModel();

  /// 计算单个刚体的回归矩阵块，接收预先计算的运动学量
  MatrixXd computeBodyRegressorBlock(
      std::size_t body_idx,
      const MatrixXd &J_origin,       // 6×n_dof body origin jacobian (WORLD frame)
      const MatrixXd &J_origin_dot,   // 6×n_dof time derivative
      const Matrix3d &R_body,         // body 世界旋转矩阵
      const Vector3d &a_local,        // body origin 加速度 (local frame)
      const Vector3d &omega_local,    // body 角速度 (local frame)
      const Vector3d &alpha_local) const;  // body 角加速度 (local frame)
};

} // namespace robot_dynamics
