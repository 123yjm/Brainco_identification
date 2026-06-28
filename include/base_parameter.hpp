#pragma once

#include <Eigen/Core>
#include <Eigen/QR>

#include <cstddef>
#include <string>
#include <vector>

namespace robot_dynamics {

// ============================================================================
// BaseParameterDecomposition — 最小惯性参数集 QR 列主元分解
// ============================================================================
//
// 对观测矩阵 W (m×n) 做 QRCP 分解，找出可辨识的参数子空间：
//
//   W · E = Q · R
//   R = [R₁  R₂]
//       [ 0   0 ]
//
// R₁ (r×r) 满秩上三角，r = rank(W)
// β_base = β₁ + R₁⁻¹·R₂·β₂  (r 维最小参数集)
// W_base = (W·E) 的前 r 列     (列满秩基回归矩阵)
//
// 用法:
//   auto dec = BaseParameterDecomposition::compute(W);
//   MatrixXd Wb = BaseParameterDecomposition::getBaseRegressor(W, dec);
//   VectorXd beta_base = solver->solve(Wb, tau);
//   VectorXd beta_full = BaseParameterDecomposition::baseToFull(beta_base, dec);

class BaseParameterDecomposition {
public:
  using MatrixXd = Eigen::MatrixXd;
  using VectorXd = Eigen::VectorXd;

  // --------------------------------------------------------------------------
  // Decomposition — QRCP 分解结果
  // --------------------------------------------------------------------------
  struct Decomposition {
    Eigen::Index rank = 0;         // 数值秩 r
    Eigen::Index n_full = 0;       // 原始参数维度 n
    double threshold = 0.0;        // 秩判定阈值
    double cond_R1 = 0.0;          // R₁ 的条件数 (≈ 最大奇异值 / 最小奇异值)

    // |R 对角线| 的绝对值（按 pivot 顺序排列）
    Eigen::VectorXd R_diag;

    // R₁ (r×r, 上三角, 满秩)
    Eigen::MatrixXd R1;

    // R₂ (r × (n-r))
    Eigen::MatrixXd R2;

    // α = R₁⁻¹·R₂ (r × (n-r)), 用于将不可辨识参数重组到可辨识参数
    // β_base = β₁ + α·β₂
    Eigen::MatrixXd alpha;

    // 列置换矩阵 E (n×n)
    Eigen::PermutationMatrix<Eigen::Dynamic> E;

    // 可辨识参数在原始 W 中的列索引 (长度 r)
    std::vector<Eigen::Index> identifiable_indices;

    // 不可辨识参数在原始 W 中的列索引 (长度 n-r)
    std::vector<Eigen::Index> unidentifiable_indices;
  };

  // --------------------------------------------------------------------------
  // compute — QRCP 分解 + 数值秩判定
  // --------------------------------------------------------------------------
  // @param W   观测矩阵 (m × n)，通常由 computeObservationMatrix 构造
  // @param tol 秩阈值乘数，默认 1.0
  //             threshold = tol * ε * max(m,n) * abs(R(0,0))
  //             增大 tol → 更保守 → 更多列被判为不可辨识
  // @return    Decomposition 结构体
  static Decomposition compute(const MatrixXd &W, double tol = 100.0);

  // --------------------------------------------------------------------------
  // getBaseRegressor — 提取列满秩的基回归矩阵
  // --------------------------------------------------------------------------
  // W_base = (W * E).leftCols(rank)  维度: m × r
  // 列满秩，可直接用于最小二乘求解
  static MatrixXd getBaseRegressor(const MatrixXd &W,
                                   const Decomposition &decomp);

  // --------------------------------------------------------------------------
  // baseToFull — 最小参数集映射回全参数空间（最小范数解）
  // --------------------------------------------------------------------------
  // 令 β₂ = 0（不可辨识参数设为零），则:
  //   β₁ = β_base
  //   β_full = E · [β₁; 0]
  // 这给出满足 τ = W·β_full 的最小 ||β|| 解
  static VectorXd baseToFull(const VectorXd &beta_base,
                             const Decomposition &decomp);

  // --------------------------------------------------------------------------
  // baseToFullWithPrior — 用先验值填充不可辨识参数，从数据求解可辨识参数
  // --------------------------------------------------------------------------
  // β_base = p₁ + α·p₂  (已知 β_base 和 p₂ = E^T·β_prior 的不可辨识部分)
  // → p₁ = β_base - α·p₂
  // → β_full = E · [p₁; p₂]
  // 可辨识参数由数据决定，不可辨识参数保持先验值
  static VectorXd baseToFullWithPrior(const VectorXd &beta_base,
                                      const VectorXd &beta_prior_full,
                                      const Decomposition &decomp);

  // --------------------------------------------------------------------------
  // getNullVector — 生成零空间向量
  // --------------------------------------------------------------------------
  // 对任意 v ∈ R^{n-r}，构造 null = E · [α·v; -v]
  // 满足 W · null = 0（机器精度内）
  // 用于验证分解正确性
  static VectorXd getNullVector(const VectorXd &v,
                                const Decomposition &decomp);

  // --------------------------------------------------------------------------
  // getCombinationMatrix — 获取组合矩阵 C (r×n)
  // --------------------------------------------------------------------------
  // 满足 β_base = C · β_full
  // C 的第 i 行给出了第 i 个基参数由哪些原始物理参数组合而成:
  //   β_base[i] = C(i, idx_i) * β_full[idx_i] + Σ_j C(i, idx_j) * β_full[idx_j]
  // 其中 C(i, identifiable_indices[i]) = 1.0 (主参数)
  //       C(i, unidentifiable_indices[j]) = alpha(i,j) (重组系数)
  static MatrixXd getCombinationMatrix(const Decomposition &decomp);

  // --------------------------------------------------------------------------
  // getBaseParameterFormulas — 基参数组合公式（可读文本）
  // --------------------------------------------------------------------------
  // 每行格式: "base[ 0] = +1.000000*link7_mz + 0.523000*link1_m + (-0.134000)*link2_Iyy"
  // 只输出 |系数| > 1e-10 的项，主参数（系数=1）排第一
  static std::vector<std::string> getBaseParameterFormulas(
      const Decomposition &decomp,
      const std::vector<std::string> &full_param_names);
};

} // namespace robot_dynamics
