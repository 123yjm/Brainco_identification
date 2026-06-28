#include "base_parameter.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace robot_dynamics {

// ============================================================================
// compute — QRCP 分解 + 数值秩判定
// ============================================================================

BaseParameterDecomposition::Decomposition
BaseParameterDecomposition::compute(const MatrixXd &W, double tol) {
  const Eigen::Index m = W.rows();
  const Eigen::Index n = W.cols();

  if (m == 0 || n == 0) {
    throw std::runtime_error(
        "BaseParameterDecomposition::compute: W is empty (" +
        std::to_string(m) + "x" + std::to_string(n) + ")");
  }

  // ---- 列缩放 (参考 robotdynid: 每列除以 L2 范数, 消除量纲差异) ---------------
  Eigen::VectorXd scales(n);
  for (Eigen::Index j = 0; j < n; ++j) {
    double norm = W.col(j).norm();
    scales(j) = (norm > 0.0) ? norm : 1.0;
  }
  MatrixXd W_scaled = W;
  for (Eigen::Index j = 0; j < n; ++j) {
    W_scaled.col(j) /= scales(j);
  }

  // ---- QR 列主元分解 (在缩放后的矩阵上) ----------------------------------------
  Eigen::ColPivHouseholderQR<MatrixXd> qr(W_scaled);

  // R_raw 是 m×n，只有 top-left min(m,n)×n 部分有意义
  Eigen::Index n_rows = std::min(m, n);
  MatrixXd R_raw = qr.matrixR().topRows(n_rows);

  // 提取 |R 对角线|（按 pivot 顺序）
  Eigen::VectorXd R_diag(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    R_diag(i) = std::abs(R_raw(i, i));
  }

  // ---- 数值秩判定 ------------------------------------------------------------
  double eps = std::numeric_limits<double>::epsilon();
  double threshold = tol * eps * static_cast<double>(std::max(m, n)) * R_diag(0);

  Eigen::Index rank = 0;
  for (Eigen::Index i = 0; i < n; ++i) {
    if (R_diag(i) > threshold)
      ++rank;
    else
      break; // pivot 按降序排列，一旦跌破阈值后面都不可辨识
  }

  if (rank == 0) {
    throw std::runtime_error(
        "BaseParameterDecomposition::compute: rank = 0. "
        "Threshold = " + std::to_string(threshold) +
        ", R_diag(0) = " + std::to_string(R_diag(0)));
  }

  Eigen::Index deficiency = n - rank;

  // ---- 提取 R1 & R2 ---------------------------------------------------------
  MatrixXd R1 = R_raw.topLeftCorner(rank, rank).template triangularView<Eigen::Upper>();
  MatrixXd R2 = R_raw.block(0, rank, rank, deficiency);

  // ---- α_scaled = R1^{-1} * R2 (缩放空间中的依赖矩阵) ------------------------
  MatrixXd alpha_scaled = MatrixXd::Zero(rank, deficiency);
  if (deficiency > 0) {
    alpha_scaled = R1.template triangularView<Eigen::Upper>().solve(R2);
  }

  // ---- 反缩放: α = diag(1/s_keep) · α_scaled · diag(s_dep) ---------------
  //   Y_dep / s_dep = (Y_keep / s_keep) · α_scaled
  //   → Y_dep = Y_keep · diag(1/s_keep) · α_scaled · diag(s_dep)
  //   → α_unscaled(i,j) = α_scaled(i,j) * s_dep(j) / s_keep(i)
  MatrixXd alpha = MatrixXd::Zero(rank, deficiency);
  if (deficiency > 0) {
    // 先提取 keep/dep 对应的缩放因子
    Eigen::VectorXd s_keep(rank), s_dep(deficiency);
    const int *piv = qr.colsPermutation().indices().data();
    for (Eigen::Index i = 0; i < rank; ++i)
      s_keep(i) = scales(static_cast<Eigen::Index>(piv[i]));
    for (Eigen::Index j = 0; j < deficiency; ++j)
      s_dep(j) = scales(static_cast<Eigen::Index>(piv[rank + j]));

    for (Eigen::Index i = 0; i < rank; ++i) {
      for (Eigen::Index j = 0; j < deficiency; ++j) {
        alpha(i, j) = alpha_scaled(i, j) * s_dep(j) / s_keep(i);
      }
    }
  }

  // ---- 条件数估计 -----------------------------------------------------------
  double cond_R1 = R_diag(0) / R_diag(rank - 1);

  // ---- 置换信息 -------------------------------------------------------------
  const auto &perm = qr.colsPermutation();
  std::vector<Eigen::Index> identifiable(rank);
  std::vector<Eigen::Index> unidentifiable(deficiency);
  const int *indices = perm.indices().data();
  for (Eigen::Index i = 0; i < rank; ++i)
    identifiable[static_cast<std::size_t>(i)] = static_cast<Eigen::Index>(indices[i]);
  for (Eigen::Index i = 0; i < deficiency; ++i)
    unidentifiable[static_cast<std::size_t>(i)] = static_cast<Eigen::Index>(indices[rank + i]);

  // ---- 组装结果 -------------------------------------------------------------
  Decomposition dec;
  dec.rank = rank;
  dec.n_full = n;
  dec.threshold = threshold;
  dec.cond_R1 = cond_R1;
  dec.R_diag = std::move(R_diag);
  dec.R1 = std::move(R1);
  dec.R2 = std::move(R2);
  dec.alpha = std::move(alpha);
  dec.E = perm;
  dec.identifiable_indices = std::move(identifiable);
  dec.unidentifiable_indices = std::move(unidentifiable);

#ifdef IDENTIFICATION_VERBOSE
  std::cout << "[BaseParameterDecomposition] QRCP: " << m << "×" << n
            << " → rank=" << rank << ", deficiency=" << deficiency
            << ", cond(R1)=" << cond_R1
            << ", threshold=" << threshold << "\n";
#endif

  return dec;
}

// ============================================================================
// getBaseRegressor — 提取基回归矩阵
// ============================================================================

BaseParameterDecomposition::MatrixXd
BaseParameterDecomposition::getBaseRegressor(const MatrixXd &W,
                                             const Decomposition &decomp) {
  // W_base = (W * E).leftCols(rank)
  // 等价于 W * E 的前 rank 列
  // 高效做法：直接取 W 中被置换到前 rank 的列
  MatrixXd W_base(W.rows(), decomp.rank);
  for (Eigen::Index j = 0; j < decomp.rank; ++j) {
    W_base.col(j) = W.col(decomp.identifiable_indices[static_cast<std::size_t>(j)]);
  }
  return W_base;
}

// ============================================================================
// baseToFull — 映射回全参数空间
// ============================================================================

BaseParameterDecomposition::VectorXd
BaseParameterDecomposition::baseToFull(const VectorXd &beta_base,
                                       const Decomposition &decomp) {
  const Eigen::Index n = decomp.n_full;
  const Eigen::Index r = decomp.rank;

  // β_base = p₁ + α·p₂（其中 p = E^T · β_full）
  // 最小范数解：令 p₂ = 0，则 p₁ = β_base
  // （OLS 在 W_base_raw = (W*E)_{:,1:r} 上的解直接给出 p₁）
  VectorXd beta1 = beta_base;

  // β₂ = 0 (最小范数)
  VectorXd beta2 = VectorXd::Zero(n - r);

  // 排列后参数: p = [β_base; 0]
  VectorXd beta_permuted(n);
  beta_permuted << beta1, beta2;

  // β_full[indices[k]] = p[k]  ⇒  β_full = P^{-1} * p = P^T * p
  // W_base 的列 j 来自 W 的列 indices[j]，所以 β_full[indices[j]] = p[j]
  VectorXd beta_full = VectorXd::Zero(n);
  const int *indices_btf = decomp.E.indices().data();
  for (Eigen::Index k = 0; k < n; ++k) {
    beta_full(static_cast<Eigen::Index>(indices_btf[k])) = beta_permuted(k);
  }

  return beta_full;
}

// ============================================================================
// baseToFullWithPrior — 用先验值填充不可辨识参数，从数据求解可辨识参数
// ============================================================================

BaseParameterDecomposition::VectorXd
BaseParameterDecomposition::baseToFullWithPrior(const VectorXd &beta_base,
                                                 const VectorXd &beta_prior_full,
                                                 const Decomposition &decomp) {
  const Eigen::Index n = decomp.n_full;
  const Eigen::Index r = decomp.rank;
  const Eigen::Index d = n - r;

  // p_prior = E^T · β_prior_full  →  p_prior[k] = β_prior_full[indices[k]]
  const int *indices = decomp.E.indices().data();
  VectorXd p_prior(n);
  for (Eigen::Index k = 0; k < n; ++k) {
    p_prior(k) = beta_prior_full(static_cast<Eigen::Index>(indices[k]));
  }

  // 不可辨识部分的先验值: p₂ = p_prior[r:n]
  VectorXd p2 = p_prior.tail(d);

  // 从 β_base = p₁ + α·p₂ 反解可辨识部分: p₁ = β_base - α·p₂
  VectorXd p1 = beta_base;
  if (d > 0) {
    p1 -= decomp.alpha * p2;
  }

  // β_full = E · [p₁; p₂]
  VectorXd p_combined(n);
  p_combined << p1, p2;

  VectorXd beta_full = VectorXd::Zero(n);
  for (Eigen::Index k = 0; k < n; ++k) {
    beta_full(static_cast<Eigen::Index>(indices[k])) = p_combined(k);
  }

  return beta_full;
}

// ============================================================================
// getNullVector — 生成零空间向量
// ============================================================================

BaseParameterDecomposition::VectorXd
BaseParameterDecomposition::getNullVector(const VectorXd &v,
                                          const Decomposition &decomp) {
  const Eigen::Index n = decomp.n_full;
  const Eigen::Index r = decomp.rank;
  const Eigen::Index d = n - r;

  if (d == 0 || v.size() != d) {
    return VectorXd::Zero(n);
  }

  // null_permuted = [α * v;  -v]
  VectorXd alpha_v = decomp.alpha * v;
  VectorXd null_permuted(n);
  null_permuted << alpha_v, -v;

  // null = E * null_permuted
  // null[indices[k]] = null_permuted[k]
  VectorXd null_vec = VectorXd::Zero(n);
  const int *indices_gnv = decomp.E.indices().data();
  for (Eigen::Index k = 0; k < n; ++k) {
    null_vec(static_cast<Eigen::Index>(indices_gnv[k])) = null_permuted(k);
  }

  return null_vec;
}

// ============================================================================
// getCombinationMatrix — 获取组合矩阵 C (r×n)
// ============================================================================

BaseParameterDecomposition::MatrixXd
BaseParameterDecomposition::getCombinationMatrix(const Decomposition &decomp) {
  const Eigen::Index r = decomp.rank;
  const Eigen::Index n = decomp.n_full;

  MatrixXd C = MatrixXd::Zero(r, n);

  const int *indices = decomp.E.indices().data();

  for (Eigen::Index i = 0; i < r; ++i) {
    // 主参数: C(i, identifiable_indices[i]) = 1
    C(i, static_cast<Eigen::Index>(indices[i])) = 1.0;

    // 重组系数: C(i, unidentifiable_indices[j]) = alpha(i, j)
    for (Eigen::Index j = 0; j < n - r; ++j) {
      double a = decomp.alpha(i, j);
      if (std::abs(a) > 1e-15) {
        C(i, static_cast<Eigen::Index>(indices[r + j])) = a;
      }
    }
  }

  return C;
}

// ============================================================================
// getBaseParameterFormulas — 基参数组合公式（可读文本）
// ============================================================================

std::vector<std::string>
BaseParameterDecomposition::getBaseParameterFormulas(
    const Decomposition &decomp,
    const std::vector<std::string> &full_param_names) {

  const Eigen::Index r = decomp.rank;
  const Eigen::Index n = decomp.n_full;
  const Eigen::Index d = n - r;

  const int *indices = decomp.E.indices().data();

  std::vector<std::string> formulas;
  formulas.reserve(static_cast<std::size_t>(r));

  for (Eigen::Index i = 0; i < r; ++i) {
    std::ostringstream oss;

    // 序号
    oss << "base[" << std::setw(2) << i << "] = ";

    bool first = true;

    // 主参数 (系数 = 1)
    {
      Eigen::Index col = static_cast<Eigen::Index>(indices[i]);
      std::string name = (col < static_cast<Eigen::Index>(full_param_names.size()))
                             ? full_param_names[static_cast<std::size_t>(col)]
                             : ("col_" + std::to_string(col));
      oss << "+1.000000*" << name;
      first = false;
    }

    // 重组项 (系数 = alpha(i, j))
    for (Eigen::Index j = 0; j < d; ++j) {
      double a = decomp.alpha(i, j);
      if (std::abs(a) < 1e-10) continue; // 跳过接近零的项

      Eigen::Index col = static_cast<Eigen::Index>(indices[r + j]);
      std::string name = (col < static_cast<Eigen::Index>(full_param_names.size()))
                             ? full_param_names[static_cast<std::size_t>(col)]
                             : ("col_" + std::to_string(col));

      oss << " ";
      if (a >= 0.0) {
        oss << "+" << std::fixed << std::setprecision(6) << a << "*" << name;
      } else {
        oss << std::fixed << std::setprecision(6) << a << "*" << name; // 负数自带符号
      }
    }

    formulas.push_back(oss.str());
  }

  return formulas;
}

} // namespace robot_dynamics
