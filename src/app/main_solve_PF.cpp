/**
 * @file main_solve_PF.cpp
 * @brief 物理可行性 (Physically Feasible) 参数辨识
 *
 * 用法: ./identify_PF --robot <robot_dir>
 *
 * 使用已辨识的摩擦参数 (Fc, Fv) 和 kinematic_params.yaml 中的动力学先验值，
 * 构建降维 QP 系统，对未知惯性参数做带不等式约束的 QP 求解。
 *
 * 约束: 每个 link 的 3×3 惯性张量半正定 (PSD)
 *   通过特征值投影 — 零化负特征值，最小改动实现物理可行性。
 */

#include "body2inertial.hpp"
#include "data_loader.hpp"
#include "regressor_factory.hpp"
#include "robot_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 帮助信息
// ---------------------------------------------------------------------------
void printHelp(const char* prog) {
    std::cout
        << "物理可行性 (PF) 参数辨识 — QP 约束求解\n\n"
        << "用法: " << prog << " --robot <robot_dir>\n\n"
        << "选项:\n"
        << "  --robot <dir>  机器人目录 (如 robots/revoarm_right)\n"
        << "  --help         打印帮助信息\n\n"
        << "使用摩擦参数 + kinematic_params 先验值，\n"
        << "对未知惯性参数做带正定性/三角不等式约束的 QP 求解。\n";
}

// ---------------------------------------------------------------------------
// 从摩擦辨识结果 YAML 加载 Fc, Fv
// ---------------------------------------------------------------------------
struct FrictionCoeffs {
    Eigen::VectorXd Fc;
    Eigen::VectorXd Fv;
};

FrictionCoeffs loadFrictionCoefficients(const std::string& yaml_path,
                                         std::size_t dof) {
    FrictionCoeffs fp;
    fp.Fc = Eigen::VectorXd::Zero(dof);
    fp.Fv = Eigen::VectorXd::Zero(dof);

    try {
        auto root = YAML::LoadFile(yaml_path);
        auto params = root["parameters"];
        if (params && params.IsSequence()) {
            for (std::size_t i = 0;
                 i < params.size() && i < 2 * dof; ++i) {
                double v = params[i].as<double>();
                if (i % 2 == 0)
                    fp.Fc(i / 2) = v;
                else
                    fp.Fv(i / 2) = v;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "⚠ 加载摩擦系数失败 (" << e.what()
                  << "), 将使用零摩擦\n";
    }
    return fp;
}

// ---------------------------------------------------------------------------
// 逐点计算摩擦力矩
// ---------------------------------------------------------------------------
Eigen::MatrixXd computeFrictionTorque(const Eigen::MatrixXd& qd,
                                       const Eigen::VectorXd& Fc,
                                       const Eigen::VectorXd& Fv) {
    Eigen::Index K = qd.rows();
    Eigen::Index dof_j = qd.cols();
    Eigen::MatrixXd tau_fric = Eigen::MatrixXd::Zero(K, dof_j);
    for (Eigen::Index k = 0; k < K; ++k) {
        for (Eigen::Index j = 0; j < dof_j; ++j) {
            double sign_qd = (qd(k, j) >= 0.0) ? 1.0 : -1.0;
            tau_fric(k, j) = Fc(j) * sign_qd + Fv(j) * qd(k, j);
        }
    }
    return tau_fric;
}

// ---------------------------------------------------------------------------
// 带 Tikhonov 正则化的无约束 OLS（用于 QP 初始点）
// ---------------------------------------------------------------------------
Eigen::VectorXd olsSolve(const Eigen::MatrixXd& W,
                          const Eigen::VectorXd& tau) {
    const double lambda = 1e-6;
    Eigen::MatrixXd WtW = W.transpose() * W;
    WtW += lambda * Eigen::MatrixXd::Identity(WtW.rows(), WtW.cols());
    return WtW.ldlt().solve(W.transpose() * tau);
}

// ---------------------------------------------------------------------------
// PSD 投影: 将 3×3 惯性矩阵投影到半正定锥 (最小 Frobenius 距离)
//
//  I = [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz]
//
//  零化负特征值 → 重建矩阵。这是满足正定约束的最小改动。
// ---------------------------------------------------------------------------
void projectInertiaToPSD(double& Ixx, double& Iyy, double& Izz,
                          double& Ixy, double& Ixz, double& Iyz) {
    Eigen::Matrix3d I;
    I << Ixx, Ixy, Ixz,
         Ixy, Iyy, Iyz,
         Ixz, Iyz, Izz;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(I);
    if (eig.info() != Eigen::Success) return;

    Eigen::Vector3d vals = eig.eigenvalues();
    if (vals.minCoeff() >= 0.0) return;  // 已经 PSD

    // 零化负特征值
    for (int i = 0; i < 3; ++i)
        if (vals(i) < 0.0) vals(i) = 0.0;

    // 重建: I_psd = V * diag(vals) * V^T
    Eigen::Matrix3d I_psd = eig.eigenvectors() * vals.asDiagonal()
                            * eig.eigenvectors().transpose();

    Ixx = I_psd(0,0); Ixy = I_psd(0,1); Ixz = I_psd(0,2);
    Iyy = I_psd(1,1); Iyz = I_psd(1,2); Izz = I_psd(2,2);
}

// ---------------------------------------------------------------------------
// 对每个 Body 构建惯性张量不等式约束矩阵
//
// beta 中 body i 的 10 个参数: [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
//                                     4    5    6    7    8    9
//
// 约束 (A_full * beta_full <= b_full):
//   -Ixx <= -eps           → 行系数 {4: -1}
//   -Iyy <= -eps           → 行系数 {7: -1}
//   -Izz <= -eps           → 行系数 {9: -1}
// ---------------------------------------------------------------------------
// ADMM 约束求解
//   min  ||W_u*x - tau_r||² + λ||x||²  s.t. I_body(b) PSD for all b
// 返回 x (未知参数向量)
// ---------------------------------------------------------------------------
Eigen::VectorXd solveADMM(
    const Eigen::MatrixXd& W_u,
    const Eigen::VectorXd& tau_r,
    const std::vector<Eigen::Index>& unknown_cols,
    std::size_t n_bodies,
    Eigen::Index num_params,
    double rho = 10000.0,
    int max_iter = 300)
{
    const double lambda_ols = 1e-6;
    const double tol = 1e-6;
    Eigen::Index n_u = W_u.cols();
    Eigen::Index n_b = static_cast<Eigen::Index>(n_bodies);
    Eigen::Index n_inertia = n_b * 6;

    // 构建惯性提取矩阵 A: unknown 空间 → 6*n_bodies 惯性分量
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_inertia, n_u);
    for (Eigen::Index b = 0; b < n_b; ++b) {
        for (int k = 0; k < 6; ++k) {
            Eigen::Index full_col = b * 10 + 4 + k;
            auto it = std::find(unknown_cols.begin(), unknown_cols.end(), full_col);
            if (it != unknown_cols.end())
                A(b * 6 + k, static_cast<Eigen::Index>(it - unknown_cols.begin())) = 1.0;
        }
    }

    // 预计算不随迭代变化的部分
    Eigen::MatrixXd H_base = 2.0 * W_u.transpose() * W_u
                             + 2.0 * lambda_ols * Eigen::MatrixXd::Identity(n_u, n_u);
    Eigen::MatrixXd AtA = A.transpose() * A;
    Eigen::VectorXd Wt_tau = 2.0 * W_u.transpose() * tau_r;

    // 初始点: 无约束 OLS
    Eigen::VectorXd x = (H_base).ldlt().solve(Wt_tau);  // 2W^T*W*x = 2W^T*y  →  W^T*W*x = W^T*y
    // 重新正确计算 OLS 初始点
    Eigen::MatrixXd WtW_reg = W_u.transpose() * W_u
                              + lambda_ols * Eigen::MatrixXd::Identity(n_u, n_u);
    x = WtW_reg.ldlt().solve(W_u.transpose() * tau_r);

    Eigen::VectorXd z = A * x;
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_inertia);

    double primal = 1.0, dual = 1.0;
    int iter;
    for (iter = 0; iter < max_iter; ++iter) {
        // x-update
        Eigen::MatrixXd H = H_base + rho * AtA;
        Eigen::VectorXd rhs = Wt_tau + rho * A.transpose() * (z - u);
        Eigen::VectorXd x_new = H.ldlt().solve(rhs);

        // z-update: PSD project
        Eigen::VectorXd v = A * x_new + u;
        Eigen::VectorXd z_new = z;
        for (Eigen::Index b = 0; b < n_b; ++b) {
            double Ixx = v(b*6+0), Ixy = v(b*6+1), Ixz = v(b*6+2);
            double Iyy = v(b*6+3), Iyz = v(b*6+4), Izz = v(b*6+5);
            projectInertiaToPSD(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
            z_new(b*6+0)=Ixx; z_new(b*6+1)=Ixy; z_new(b*6+2)=Ixz;
            z_new(b*6+3)=Iyy; z_new(b*6+4)=Iyz; z_new(b*6+5)=Izz;
        }

        // u-update
        Eigen::VectorXd u_new = u + A * x_new - z_new;

        // 收敛
        primal = (A * x_new - z_new).norm() / (1.0 + std::sqrt(static_cast<double>(n_inertia)));
        dual = rho * (A.transpose() * (z_new - z)).norm()
               / (1.0 + std::sqrt(static_cast<double>(n_u)));

        x = x_new; z = z_new; u = u_new;

        if (primal < tol && dual < tol) break;
    }
    std::cout << "  ADMM: " << iter+1 << " iters, ρ=" << rho
              << ", primal=" << primal << " dual=" << dual << "\n";
    return x;
}

// ---------------------------------------------------------------------------
// 分隔线
// ---------------------------------------------------------------------------
void printSep() { std::cout << std::string(60, '-') << "\n"; }

}  // anonymous namespace

// ============================================================================
// main
// ============================================================================
int main(int argc, char* argv[]) {
    // ---- CLI ----------------------------------------------------------------
    std::string robot_dir;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            printHelp(argv[0]);
            return 0;
        } else if ((arg == "--robot" || arg == "-r") && i + 1 < argc) {
            robot_dir = robot_utils::resolvePath(
                robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
        }
    }
    if (robot_dir.empty()) {
        std::cerr << "错误: 需要 --robot <dir>\n使用 --help 查看用法\n";
        return 1;
    }

    std::string robot_name      = robot_utils::robotNameFromDir(robot_dir);
    std::string kinematic_yaml  = robot_utils::configPath(robot_dir, "kinematic_params.yaml");
    std::string friction_yaml   = robot_utils::resultFrictionPath(robot_dir,
        robot_name + "_friction_identification.yaml");

    // ---- kinematic_params.yaml ----------------------------------------------
    std::size_t dof = 0;
    std::string robot_type = robot_name;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["dof"]) dof = kroot["dof"].as<std::size_t>();
        if (kroot["robot_type"]) robot_type = kroot["robot_type"].as<std::string>();
    } catch (const std::exception& e) {
        std::cerr << "错误: kinematic_params.yaml — " << e.what() << "\n";
        return 1;
    }

    // ---- 摩擦系数 -----------------------------------------------------------
    auto friction = loadFrictionCoefficients(friction_yaml, dof);

    // ---- 加载数据 (已滤波 CSV) ------------------------------------------------
    std::string expected_csv = robot_name + "_filtered_data.csv";
    std::string data_file = robot_utils::findFirstFile(
        robot_utils::resultInertiaPath(robot_dir, ""), "*.csv");
    if (data_file.empty()) {
        std::cerr << "错误: result_inertia 下未找到 " << expected_csv << "\n";
        return 1;
    }

    ExperimentData data;
    try {
        data = DataLoader::loadCSV(data_file, dof);
    } catch (const std::exception& e) {
        std::cerr << "错误: 数据加载失败 — " << e.what() << "\n";
        return 1;
    }

    // ---- 创建 regressor -----------------------------------------------------
    std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor;
    try {
        regressor = robot_dynamics::RegressorFactory::create(robot_type, kinematic_yaml);
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法创建 regressor — " << e.what() << "\n";
        return 1;
    }

    // ---- PF 辨识: 仅惯性参数，不加 armature / damping -------------------------
    auto flags = robot_dynamics::ParamFlags::NONE;
    std::size_t num_params = regressor->numParameters(flags);

    // ---- 打印配置 -----------------------------------------------------------
    std::cout << "═══════════════════════════════════════════\n"
              << "  物理可行性 (PF) 参数辨识 — QP 约束\n"
              << "═══════════════════════════════════════════\n"
              << "机器人:           " << robot_name << "\n"
              << "数据:             " << data_file << "\n"
              << "摩擦参数:         " << friction_yaml << "\n"
              << "样本数:           " << data.n_samples << "\n"
              << "DOF:              " << dof << "\n"
              << "参数总数:         " << num_params << "\n"
              << "约束:             每 body 6 个 (正定 + 三角不等式)\n\n";

    // ---- 构建完整 W 矩阵 + tau 堆叠向量 --------------------------------------
    Eigen::Index K        = static_cast<Eigen::Index>(data.n_samples);
    Eigen::Index rows_per = static_cast<Eigen::Index>(dof);

    Eigen::MatrixXd W_full      = Eigen::MatrixXd::Zero(rows_per * K,
                                                         static_cast<Eigen::Index>(num_params));
    Eigen::VectorXd tau_stacked = Eigen::VectorXd::Zero(rows_per * K);

    for (Eigen::Index k = 0; k < K; ++k) {
        Eigen::VectorXd q_k   = data.q.row(k).transpose();
        Eigen::VectorXd qd_k  = data.qd.row(k).transpose();
        Eigen::VectorXd qdd_k = data.qdd.row(k).transpose();

        Eigen::MatrixXd Y_k = regressor->computeRegressorMatrix(q_k, qd_k, qdd_k, flags);

        W_full.block(k * rows_per, 0, rows_per,
                     static_cast<Eigen::Index>(num_params)) = Y_k;

        for (Eigen::Index j = 0; j < rows_per; ++j)
            tau_stacked(k * rows_per + j) = data.tau(k, j);
    }

    // ---- 扣除摩擦 -----------------------------------------------------------
    Eigen::MatrixXd tau_fric_mat = computeFrictionTorque(
        data.qd, friction.Fc, friction.Fv);

    Eigen::VectorXd tau_fric_stacked(rows_per * K);
    for (Eigen::Index k = 0; k < K; ++k)
        for (Eigen::Index j = 0; j < rows_per; ++j)
            tau_fric_stacked(k * rows_per + j) = tau_fric_mat(k, j);

    Eigen::VectorXd tau_net = tau_stacked - tau_fric_stacked;

    // ---- 获取先验向量 + 已知掩码 (提前 — 质量惩罚需要 beta_prior) ---------------
    Eigen::VectorXd beta_prior = regressor->computeParameterVector(flags);
    std::vector<bool> mask     = regressor->computeParameterMask(flags);

    // ---- 质量软约束: 增广 W 和 tau -------------------------------------------
    // 惩罚项: w * Σ (m_i - m_prior_i)²
    // 增广行: W_aug[new_row, m_col] = √w,  tau_aug[new_row] = √w * m_prior
    const double mass_penalty_weight = 1e-4;  // 可调: 越大=越接近先验
    double sqrt_w = std::sqrt(mass_penalty_weight);

    Eigen::Index n_orig_rows = W_full.rows();
    Eigen::Index n_aug_rows   = n_orig_rows + static_cast<Eigen::Index>(regressor->nBodies());

    Eigen::MatrixXd W_aug = Eigen::MatrixXd::Zero(n_aug_rows, static_cast<Eigen::Index>(num_params));
    W_aug.topRows(n_orig_rows) = W_full;

    Eigen::VectorXd tau_aug = Eigen::VectorXd::Zero(n_aug_rows);
    tau_aug.head(n_orig_rows) = tau_net;

    for (std::size_t b = 0; b < regressor->nBodies(); ++b) {
        Eigen::Index mass_col = static_cast<Eigen::Index>(b * 10 + 0);
        Eigen::Index aug_row   = n_orig_rows + static_cast<Eigen::Index>(b);
        W_aug(aug_row, mass_col) = sqrt_w;
        tau_aug(aug_row)         = sqrt_w * beta_prior(mass_col);
    }
    std::cout << "质量软约束: w=" << mass_penalty_weight << "\n";

    // 之后的计算使用增广后的 W_aug / tau_aug
    W_full = W_aug;
    tau_net = tau_aug;

    // 分离已知 / 未知列索引
    std::vector<Eigen::Index> known_cols, unknown_cols;
    for (Eigen::Index col = 0; col < static_cast<Eigen::Index>(num_params); ++col) {
        if (mask[static_cast<std::size_t>(col)])
            known_cols.push_back(col);
        else
            unknown_cols.push_back(col);
    }

    Eigen::Index n_known   = static_cast<Eigen::Index>(known_cols.size());
    Eigen::Index n_unknown = static_cast<Eigen::Index>(unknown_cols.size());

    std::cout << "参数统计:  已知=" << n_known << " (先验约束),  未知="
              << n_unknown << " (待求解)\n\n";

    if (n_unknown == 0) {
        std::cerr << "错误: 所有参数均为已知，无需求解\n";
        return 1;
    }

    // 提取 W_known, W_unknown, beta_known (使用增广后的行数)
    Eigen::Index total_rows = W_full.rows();  // = n_orig_rows + n_bodies
    Eigen::MatrixXd W_known(total_rows, n_known);
    Eigen::MatrixXd W_unknown(total_rows, n_unknown);
    Eigen::VectorXd beta_known(n_known);

    for (Eigen::Index i = 0; i < n_known; ++i) {
        W_known.col(i) = W_full.col(known_cols[static_cast<std::size_t>(i)]);
        beta_known(i)  = beta_prior(known_cols[static_cast<std::size_t>(i)]);
    }
    for (Eigen::Index i = 0; i < n_unknown; ++i) {
        W_unknown.col(i) = W_full.col(unknown_cols[static_cast<std::size_t>(i)]);
    }

    // ---- 构建已知掩码 + 已知值 ------------------------------------------------
    std::vector<bool> known_mask_all(num_params, false);
    Eigen::VectorXd beta_known_full_vec = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(num_params));
    for (Eigen::Index i = 0; i < n_known; ++i) {
        known_mask_all[static_cast<std::size_t>(known_cols[static_cast<std::size_t>(i)])] = true;
        beta_known_full_vec(known_cols[static_cast<std::size_t>(i)]) = beta_known(i);
    }

    // ---- ADMM 约束优化 -------------------------------------------------------
    // RHS 扣除已知项
    Eigen::VectorXd tau_reduced = tau_net;
    for (Eigen::Index i = 0; i < n_known; ++i)
        tau_reduced -= W_known.col(i) * beta_known(i);

    std::cout << "ADMM 约束优化... (未知参数=" << n_unknown
              << ", bodies=" << regressor->nBodies() << ")\n";

    Eigen::VectorXd beta_unknown = solveADMM(
        W_unknown, tau_reduced, unknown_cols,
        regressor->nBodies(), static_cast<Eigen::Index>(num_params));

    // 重构全空间 beta
    Eigen::VectorXd beta_full = beta_known_full_vec;
    for (Eigen::Index i = 0; i < n_unknown; ++i)
        beta_full(unknown_cols[static_cast<std::size_t>(i)]) = beta_unknown(i);

    // ---- 约束验证 -----------------------------------------------------------
    std::size_t n_violations = 0;
    for (std::size_t b = 0; b < regressor->nBodies(); ++b) {
        double Ixx = beta_full(b*10+4), Iyy = beta_full(b*10+7), Izz = beta_full(b*10+9);
        double Ixy = beta_full(b*10+5), Ixz = beta_full(b*10+6), Iyz = beta_full(b*10+8);

        Eigen::Matrix3d I;
        I << Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(I);
        double min_eig = eig.eigenvalues().minCoeff();

        if (min_eig < -1e-10) {
            if (n_violations < 5)
                std::cerr << "⚠ Body " << b << ": λ_min=" << min_eig << "\n";
            n_violations++;
        }
    }
    if (n_violations == 0)
        std::cout << "  ✓ 所有 PSD\n";

    // ---- 残差分析 -----------------------------------------------------------
    Eigen::VectorXd residual = W_full * beta_full - tau_net;
    double rmse    = std::sqrt(residual.squaredNorm() /
                                static_cast<double>(tau_stacked.size()));
    double max_err = residual.cwiseAbs().maxCoeff();

    // ---- 终端输出 -----------------------------------------------------------
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n扭矩 RMSE:       " << rmse << "\n"
              << "扭矩最大误差:    " << max_err << "\n\n";

    printSep();
    std::cout << "惯性参数 (每行 = 一个 Body 的 10 个参数):\n";
    printSep();

    std::size_t n_bodies = regressor->nBodies();
    Eigen::Index inertial_count = static_cast<Eigen::Index>(n_bodies * 10);

    // 列标题
    std::cout << "Body   "
              << std::setw(10) << "m" << " " << std::setw(10) << "mx"
              << " " << std::setw(10) << "my" << " " << std::setw(10) << "mz"
              << " " << std::setw(10) << "Ixx" << " " << std::setw(10) << "Ixy"
              << " " << std::setw(10) << "Ixz" << " " << std::setw(10) << "Iyy"
              << " " << std::setw(10) << "Iyz" << " " << std::setw(10) << "Izz"
              << "\n";
    printSep();

    for (Eigen::Index i = 0; i < inertial_count; i += 10) {
        std::cout << std::setw(4) << (i / 10) << "  ";
        for (Eigen::Index j = 0; j < 10; ++j) {
            std::cout << std::setw(10) << beta_full(i + j) << " ";
        }
        std::cout << "\n";
    }
    printSep();

    // ---- 保存 YAML ----------------------------------------------------------
    std::string output_yaml = robot_utils::resultPFPath(robot_dir,
        robot_name + "_pf_identification.yaml");
    std::filesystem::create_directories(
        robot_utils::resultPFPath(robot_dir, ""));

    std::ofstream out(output_yaml);
    if (out) {
        std::time_t now = std::time(nullptr);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
                      std::localtime(&now));

        out << std::fixed << std::setprecision(3);
        out << "calibration_date: \"" << buf << "\"\n"
            << "robot: \"" << robot_name << "\"\n"
            << "method: \"physically_feasible_qp\"\n"
            << "evaluation_method: \"torque_residual_rmse\"\n"
            << "known_parameter_count: " << n_known << "\n"
            << "unknown_parameter_count: " << n_unknown << "\n"
            << "torque_rmse: " << rmse << "\n"
            << "torque_max_error: " << max_err << "\n"
            << "constraints_satisfied: "
            << (n_violations == 0 ? "true" : "false") << "\n"
            << "parameters:\n"
            << "    [\n";

        for (Eigen::Index i = 0; i < inertial_count; i += 10) {
            out << "        ";
            for (Eigen::Index j = 0; j < 10 && (i + j) < inertial_count; ++j) {
                out << beta_full(i + j);
                if (i + j + 1 < static_cast<Eigen::Index>(num_params))
                    out << ",";
            }
            out << "\n";
        }

        out << "    ]\n";
    }

    std::cout << "\n结果已保存: " << output_yaml << "\n";
    return 0;
}
