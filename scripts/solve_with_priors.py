#!/usr/bin/env python3
"""
基于不可辨识参数的先验值 + 基参数方程，精确求解全参数。

核心公式 (QRCP):
  p = E^T · β_full       (排列后参数)
  p₁ = p[0:r], p₂ = p[r:n]   (可辨识/不可辨识)
  β_base = p₁ + α · p₂        (基参数)

已知 p₂ (来自先验) 和 β_base (来自 OLS):
  p₁ = β_base - α · p₂        (精确)
  β_full = E · [p₁; p₂]       (精确)

用法:
  python3 scripts/solve_with_priors.py --robot robots/revoarm_right
"""

import argparse, os, sys
import numpy as np
try:
    import yaml
except ImportError:
    print("需要 PyYAML: pip install pyyaml"); sys.exit(1)

PARAM_NAMES = ["m", "mx", "my", "mz", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz"]


def resolve_paths(robot_dir):
    name = os.path.basename(os.path.normpath(robot_dir))
    base = os.path.normpath(robot_dir)
    return {
        "robot_name": name,
        "prior_yaml": os.path.join(base, "config", "dynamic_prior_params.yaml"),
        "base_yaml": os.path.join(base, "result_inertia", f"{name}_base_inertia_identification.yaml"),
    }


def load_prior(prior_yaml):
    with open(prior_yaml) as f:
        data = yaml.safe_load(f)
    bodies = data["bodies"]
    n = len(bodies) * 10
    beta = np.full(n, np.nan)
    names = []
    for i, body in enumerate(bodies):
        ip = body["inertial_params"]
        for j, pn in enumerate(PARAM_NAMES):
            idx = i * 10 + j
            val = ip[pn]
            if val is not None:
                beta[idx] = float(val)
                names.append(f"{body['name']}_{pn}")
    return beta, ~np.isnan(beta), [b["name"] for b in bodies]


def load_base(base_yaml):
    with open(base_yaml) as f:
        data = yaml.safe_load(f)
    cm = data["combination_matrix"]
    C = np.array(cm["matrix_C"]).reshape(cm["rows"], cm["cols"])
    beta_base = np.array(data["base_solution"]["base_parameters"])
    # 不可辨识参数索引 (来自 QRCP)
    unid_idx = data["unidentifiable_parameters"]["indices"]
    return C, beta_base, data, cm, unid_idx


def exact_solve(C, beta_base, r, n, id_idx, unid_idx, alpha, known_mask, beta_prior):
    """
    精确求解: p₂ 来自先验, p₁ = β_base - α·p₂, β_full = E·[p₁;p₂]
    前提: 所有不可辨识参数都有先验值
    """
    # 排列索引: 可辨识在前, 不可辨识在后
    E_indices = np.array(list(id_idx) + list(unid_idx), dtype=int)

    # p₂: 不可辨识参数在原始空间的值
    unid_orig_idx = E_indices[r:]  # 后 13 个排列索引 → 原始空间
    p2 = beta_prior[unid_orig_idx]

    if np.any(np.isnan(p2)):
        missing = np.sum(np.isnan(p2))
        raise ValueError(f"{missing} 个不可辨识参数仍缺先验: {unid_orig_idx[np.isnan(p2)]}")

    # α 矩阵已作为参数传入
    alpha_mat = alpha.reshape(alpha.shape[0], alpha.shape[1]) if len(alpha.shape) == 1 else alpha

    # p₁ = β_base - α · p₂  (精确)
    p1 = beta_base - alpha_mat @ p2

    # 排列后参数
    p = np.concatenate([p1, p2])

    # β_full = E · p  (反排列)
    beta_full = np.zeros(n)
    for k in range(n):
        beta_full[E_indices[k]] = p[k]

    # 覆盖已知参数为先验精确值 (消除浮点误差)
    beta_full[known_mask] = beta_prior[known_mask]

    # 验证
    b_pred = C @ beta_full
    residual = b_pred - beta_base
    rmse = np.sqrt(np.mean(residual**2))
    max_err = np.max(np.abs(residual))

    return beta_full, p1, p2, rmse, max_err


def save_yaml(base_yaml, beta_full, body_names, known_mask, rmse, max_err):
    with open(base_yaml) as f:
        original = f.read()

    n_known = int(np.sum(known_mask))
    n_solved = len(beta_full) - n_known

    sec = f"""

# ==============================================================================
# prior_informed_solution — 基于不可辨识参数先验的精确求解
# ==============================================================================
prior_informed_solution:
  method: "qr_decomposition_exact_solve"
  description: >
    不可辨识参数 p₂ 由 MuJoCo XML 先验给定,
    可辨识参数 p₁ = β_base - α·p₂ 精确计算,
    β_full = E·[p₁;p₂] 反排列.
    满足 C·β_full = β_base (机器精度).
  known_params_fixed: {n_known}
  solved_params: {n_solved}
  base_param_residual_rmse: {rmse:.6e}
  base_param_residual_max: {max_err:.6e}
  full_parameters:
"""
    n_bodies = len(body_names)
    for b in range(n_bodies):
        name = body_names[b]
        vals = beta_full[b*10:(b+1)*10]
        sec += f"    - [" + ", ".join(f"{v:.8e}" for v in vals) + f"]  # {name}\n"

    sec += f"\n  parameter_sources:\n"
    for b in range(n_bodies):
        name = body_names[b]
        srcs = []
        for j, pn in enumerate(PARAM_NAMES):
            idx = b*10 + j
            srcs.append(f"{pn}: prior" if known_mask[idx] else f"{pn}: solved")
        sec += f"    {name}:\n"
        for s in srcs:
            sec += f"      {s}\n"

    with open(base_yaml, "w") as f:
        f.write(original.rstrip() + "\n" + sec)
    print(f"结果已追加到: {base_yaml}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot", "-r", required=True)
    args = p.parse_args()
    paths = resolve_paths(args.robot)

    print(f"加载先验: {paths['prior_yaml']}")
    beta_prior, known_mask, body_names = load_prior(paths["prior_yaml"])
    n_total = len(beta_prior)
    n_known = int(np.sum(known_mask))
    print(f"  全参数: {n_total}, 先验固定: {n_known}, 待求解: {n_total-n_known}")

    print(f"加载基参数: {paths['base_yaml']}")
    C, beta_base, data, cm, unid_idx = load_base(paths["base_yaml"])
    r, n = C.shape
    print(f"  C: {r}×{n}, rank={r}")

    # 检查: 所有不可辨识参数是否都有先验?
    unid_missing = [i for i in unid_idx if not known_mask[i]]
    if unid_missing:
        print(f"\n  ⚠ {len(unid_missing)} 个不可辨识参数缺先验:")
        all_names = data["identifiable_parameters"]["names"] + data["unidentifiable_parameters"]["names"]
        # 重新构建完整名称列表
        full_names_list = []
        for b in body_names:
            for pn in PARAM_NAMES:
                full_names_list.append(f"{b}_{pn}")
        for i in unid_missing:
            nm = full_names_list[i] if i < len(full_names_list) else f"col_{i}"
            print(f"    [{i}] {nm}")
        sys.exit(1)

    print(f"\n  所有 {len(unid_idx)} 个不可辨识参数均有先验 ✓")

    # 精确求解
    print("精确求解 p₁ = β_base - α·p₂ ...")
    dec = data["decomposition"]
    id_idx = data["identifiable_parameters"]["indices"]
    unid_idx = data["unidentifiable_parameters"]["indices"]
    rm = data["regrouping_map"]
    alpha = np.array(rm["alpha_values"]).reshape(rm["alpha_rows"], rm["alpha_cols"])
    beta_full, p1, p2, rmse, max_err = exact_solve(
        C, beta_base, dec["rank"], dec["parameters_full"],
        id_idx, unid_idx, alpha, known_mask, beta_prior)

    print(f"  基参数残差 RMSE: {rmse:.6e}")
    print(f"  基参数残差 Max:  {max_err:.6e}")

    # 输出
    n_bodies = len(body_names)
    print(f"\n{'='*70}")
    print(f"  求解结果 ({n_bodies} body × 10)")
    print(f"{'='*70}")
    for b in range(n_bodies):
        vals = beta_full[b*10:(b+1)*10]
        print(f"\n  {body_names[b]}:")
        for j, pn in enumerate(PARAM_NAMES):
            idx = b*10 + j
            tag = "(prior)" if known_mask[idx] else "(solved)"
            print(f"    {pn:4s} = {vals[j]: 12.6f}  {tag}")

    # 物理检查
    print(f"\n{'='*70}")
    print(f"  物理合理性检查")
    print(f"{'='*70}")
    issues = 0
    for b in range(n_bodies):
        name = body_names[b]
        m = beta_full[b*10]
        Ixx = beta_full[b*10+4]; Iyy = beta_full[b*10+7]; Izz = beta_full[b*10+9]
        if m <= 0:          print(f"  ✗ {name}: m={m:.4f} ≤ 0"); issues+=1
        if Ixx <= 0:        print(f"  ✗ {name}: Ixx={Ixx:.6f} ≤ 0"); issues+=1
        if Iyy <= 0:        print(f"  ✗ {name}: Iyy={Iyy:.6f} ≤ 0"); issues+=1
        if Izz <= 0:        print(f"  ✗ {name}: Izz={Izz:.6f} ≤ 0"); issues+=1
        if Ixx>0 and Iyy>0 and Izz>0:
            if Ixx+Iyy < Izz-1e-10: print(f"  ✗ {name}: Ixx+Iyy < Izz"); issues+=1
            if Iyy+Izz < Ixx-1e-10: print(f"  ✗ {name}: Iyy+Izz < Ixx"); issues+=1
            if Izz+Ixx < Iyy-1e-10: print(f"  ✗ {name}: Izz+Ixx < Iyy"); issues+=1
    if issues == 0:
        print("  所有参数物理合理 ✓")

    save_yaml(paths["base_yaml"], beta_full, body_names, known_mask, rmse, max_err)


if __name__ == "__main__":
    main()
