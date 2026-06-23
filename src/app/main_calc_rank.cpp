/**
 * @file main_calc_rank.cpp
 * @brief 计算观测矩阵 W 的数值秩与奇异值谱
 *
 * 用法: ./calc_rank --robot <robot_dir> [选项]
 *
 * 通过 SVD 分解计算 W = [Y_1; Y_2; ...; Y_K] 的数值秩，
 * 报告秩亏、条件数和完整的奇异值分布，用于评估参数可辨识性。
 */

#include "data_loader.hpp"
#include "regressor_factory.hpp"
#include "robot_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// 输出分隔线
// ---------------------------------------------------------------------------
void printSep() { std::cout << std::string(60, '-') << "\n"; }

// ---------------------------------------------------------------------------
// 帮助信息
// ---------------------------------------------------------------------------
void printHelp(const char* prog) {
    std::cout
        << "观测矩阵秩分析\n\n"
        << "用法: " << prog << " --robot <robot_dir> [选项]\n\n"
        << "选项:\n"
        << "  --robot <dir>         机器人目录 (如 robots/revoarm_right)\n"
        << "  --armature            启用 armature 列\n"
        << "  --no-armature         禁用 armature 列\n"
        << "  --damping             启用 damping 列\n"
        << "  --no-damping          禁用 damping 列\n"
        << "  --no-cross-inertia    剔除每个 body 的 Ixy/Ixz/Iyz 列\n"
        << "                          (每 body 从 10 列减为 7 列)\n"
        << "  --tol <value>         秩判定阈值乘数 (默认 1.0)\n"
        << "                          秩阈值 = tol * eps * max(m,n) * sigma_max\n"
        << "  --help                打印帮助信息\n\n"
        << "若未指定 armature/damping，默认从 identification.yaml 读取。\n";
}

// ---------------------------------------------------------------------------
// 结构体：命令行选项
// ---------------------------------------------------------------------------
struct Options {
    std::string robot_dir;
    bool armature_set = false;
    bool armature = true;
    bool damping_set = false;
    bool damping = true;
    bool no_cross_inertia = false;
    double tol_multiplier = 1.0;
};

// ---------------------------------------------------------------------------
// 构建列掩码：根据 --no-cross-inertia 标记，筛选需要保留的列索引
//
// 惯性参数每 body 10 列: [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
//                                       idx: 0   1   2   3    4    5    6    7    8    9
// --no-cross-inertia 时剔除 Ixy(5), Ixz(6), Iyz(8)，保留 7 列
//
// 返回 {keep_indices, reduced_cols}
// ---------------------------------------------------------------------------
std::pair<std::vector<Eigen::Index>, Eigen::Index>
buildColumnMask(const Options& opt, std::size_t n_bodies, std::size_t n_dof,
                robot_dynamics::ParamFlags flags) {
    using robot_dynamics::hasFlag;
    using robot_dynamics::ParamFlags;

    std::vector<Eigen::Index> keep;
    constexpr int PPB = 10;  // PARAMS_PER_BODY

    // 惯性部分
    for (std::size_t b = 0; b < n_bodies; ++b) {
        Eigen::Index base = static_cast<Eigen::Index>(b * PPB);
        // 每 body 10 列中的保留列索引
        const int keep_local[] = {0, 1, 2, 3, 4, 7, 9};  // 去掉 Ixy(5), Ixz(6), Iyz(8)
        if (opt.no_cross_inertia) {
            for (int idx : keep_local)
                keep.push_back(base + idx);
        } else {
            for (int j = 0; j < PPB; ++j)
                keep.push_back(base + j);
        }
    }

    Eigen::Index next_offset = static_cast<Eigen::Index>(n_bodies * PPB);

    // armature
    if (hasFlag(flags, ParamFlags::ARMATURE)) {
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(n_dof); ++j)
            keep.push_back(next_offset + j);
        next_offset += static_cast<Eigen::Index>(n_dof);
    }

    // damping
    if (hasFlag(flags, ParamFlags::DAMPING)) {
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(n_dof); ++j)
            keep.push_back(next_offset + j);
    }

    return {keep, static_cast<Eigen::Index>(keep.size())};
}

}  // anonymous namespace

// ============================================================================
// main
// ============================================================================
int main(int argc, char* argv[]) {
    // ---- 第一次扫描：解析 --robot / --help ---------------------------------
    Options opt;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            printHelp(argv[0]);
            return 0;
        } else if ((arg == "--robot" || arg == "-r") && i + 1 < argc) {
            opt.robot_dir = robot_utils::resolvePath(
                robot_utils::resolveRobotDir(argv[++i]), PROJECT_ROOT_DIR);
        }
    }

    if (opt.robot_dir.empty()) {
        std::cerr << "错误: 需要 --robot <dir>\n"
                  << "使用 --help 查看用法\n";
        return 1;
    }

    // ---- 派生路径 ----------------------------------------------------------
    std::string robot_name = robot_utils::robotNameFromDir(opt.robot_dir);
    std::string kinematic_yaml =
        robot_utils::configPath(opt.robot_dir, "kinematic_params.yaml");
    std::string id_yaml =
        robot_utils::configPath(opt.robot_dir, "identification.yaml");
    std::string data_file =
        robot_utils::resultPath(opt.robot_dir, robot_name + "_filtered_data.csv");

    // ---- 读取 robot_type ---------------------------------------------------
    std::string robot_type = robot_name;
    try {
        auto kroot = YAML::LoadFile(kinematic_yaml);
        if (kroot["robot_type"])
            robot_type = kroot["robot_type"].as<std::string>();
    } catch (...) {}

    // ---- 读取 identification.yaml 默认值 -----------------------------------
    try {
        auto iroot = YAML::LoadFile(id_yaml);
        if (!opt.armature_set && iroot["armature"])
            opt.armature = iroot["armature"].as<bool>();
        if (!opt.damping_set && iroot["damping"])
            opt.damping = iroot["damping"].as<bool>();
    } catch (...) {}

    // ---- 第二次扫描：CLI 覆盖 -----------------------------------------------
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--armature") {
            opt.armature = true;
            opt.armature_set = true;
        } else if (arg == "--no-armature") {
            opt.armature = false;
            opt.armature_set = true;
        } else if (arg == "--damping") {
            opt.damping = true;
            opt.damping_set = true;
        } else if (arg == "--no-damping") {
            opt.damping = false;
            opt.damping_set = true;
        } else if (arg == "--no-cross-inertia") {
            opt.no_cross_inertia = true;
        } else if (arg == "--tol" && i + 1 < argc) {
            opt.tol_multiplier = std::stod(argv[++i]);
            if (opt.tol_multiplier <= 0.0) {
                std::cerr << "错误: --tol 必须 > 0\n";
                return 1;
            }
        }
    }

    // ---- 创建 regressor ----------------------------------------------------
    std::cout << "机器人: " << robot_name << " (type: " << robot_type << ")"
              << "\n运动学: " << kinematic_yaml << std::endl;

    std::unique_ptr<robot_dynamics::IDynamicsRegressor> regressor;
    try {
        regressor =
            robot_dynamics::RegressorFactory::create(robot_type, kinematic_yaml);
    } catch (const std::exception& e) {
        std::cerr << "错误: 无法创建 regressor — " << e.what() << std::endl;
        return 1;
    }

    // ---- 设置 ParamFlags ---------------------------------------------------
    auto flags = robot_dynamics::ParamFlags::NONE;
    if (opt.armature) flags = flags | robot_dynamics::ParamFlags::ARMATURE;
    if (opt.damping)  flags = flags | robot_dynamics::ParamFlags::DAMPING;

    const std::size_t n_dof = regressor->nDof();
    const std::size_t n_bodies = regressor->nBodies();
    const std::size_t num_params_full = regressor->numParameters(flags);

    // ---- 列掩码 ------------------------------------------------------------
    auto [keep_cols, n_cols_reduced] = buildColumnMask(opt, n_bodies, n_dof, flags);

    // ---- 参数配置摘要 -------------------------------------------------------
    printSep();
    std::cout << "参数配置:\n"
              << "  Bodies:          " << n_bodies
              << "  (每 body " << (opt.no_cross_inertia ? 7 : 10) << " 列)\n"
              << "  Cross-inertia:   " << (opt.no_cross_inertia ? "OFF (剔除 Ixy/Ixz/Iyz)" : "ON" ) << "\n"
              << "  Armature:        " << (opt.armature ? "ON  (+" + std::to_string(n_dof) + ")" : "OFF") << "\n"
              << "  Damping:         " << (opt.damping  ? "ON  (+" + std::to_string(n_dof) + ")" : "OFF") << "\n"
              << "  全参数列数:      " << num_params_full << "\n"
              << "  有效列数 (裁剪后): " << n_cols_reduced << std::endl;

    // ---- 加载数据 -----------------------------------------------------------
    std::cout << "数据文件: " << data_file << std::endl;

    ExperimentData data;
    try {
        data = DataLoader::loadCSV(data_file, n_dof);
    } catch (const std::exception& e) {
        std::cerr << "错误: 数据加载失败 — " << e.what() << std::endl;
        return 1;
    }

    const Eigen::Index K = static_cast<Eigen::Index>(data.n_samples);
    const Eigen::Index rows_per = static_cast<Eigen::Index>(n_dof);
    const Eigen::Index n_cols_full = static_cast<Eigen::Index>(num_params_full);
    const Eigen::Index n_rows = rows_per * K;

    std::cout << "样本数: " << K << ", DOF: " << n_dof
              << "\nW 全矩阵维度: " << n_rows << " x " << n_cols_full << std::endl;

    // ---- 构造观测矩阵 W_full，再裁剪为 W ------------------------------------
    std::cout << "构造观测矩阵 W ..." << std::endl;

    Eigen::MatrixXd W_full = Eigen::MatrixXd::Zero(n_rows, n_cols_full);

    for (Eigen::Index k = 0; k < K; ++k) {
        Eigen::VectorXd q   = data.q.row(k).transpose();
        Eigen::VectorXd qd  = data.qd.row(k).transpose();
        Eigen::VectorXd qdd = data.qdd.row(k).transpose();
        Eigen::MatrixXd Y_k = regressor->computeRegressorMatrix(q, qd, qdd, flags);
        W_full.block(k * rows_per, 0, rows_per, n_cols_full) = Y_k;
    }

    // 按列掩码裁剪
    Eigen::MatrixXd W(n_rows, n_cols_reduced);
    for (Eigen::Index j = 0; j < n_cols_reduced; ++j)
        W.col(j) = W_full.col(keep_cols[j]);

    std::cout << "裁剪后 W 维度: " << n_rows << " x " << n_cols_reduced << std::endl;

    // ---- SVD 分解 -----------------------------------------------------------
    std::cout << "SVD 分解中 ..." << std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& S = svd.singularValues();
    const Eigen::Index n_cols = n_cols_reduced;

    // ---- 数值秩判定 ---------------------------------------------------------
    double eps = std::numeric_limits<double>::epsilon();
    double threshold = opt.tol_multiplier * eps *
                       static_cast<double>(std::max(n_rows, n_cols)) * S(0);

    Eigen::Index rank = 0;
    for (Eigen::Index i = 0; i < S.size(); ++i) {
        if (S(i) > threshold) ++rank;
    }

    Eigen::Index rank_deficiency = n_cols - rank;

    // ---- 输出结果 -----------------------------------------------------------
    printSep();
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "数值秩:      " << rank << " / " << n_cols << "\n"
              << "秩亏:        " << rank_deficiency << "\n";

    if (rank > 0) {
        double cond = S(0) / S(rank - 1);
        std::cout << "条件数:      " << cond << "\n";
    }

    std::cout << "阈值 (tol):  " << threshold << "\n";
    printSep();

    // ---- 奇异值谱 -----------------------------------------------------------
    Eigen::Index head = std::min<Eigen::Index>(10, S.size());
    std::cout << "\n奇异值谱 (前 " << head << "):\n";
    for (Eigen::Index i = 0; i < head; ++i) {
        std::cout << "  sigma[" << i << "] = " << std::setw(14) << S(i);
        if (S(i) <= threshold)
            std::cout << "  <-- 秩亏方向";
        std::cout << "\n";
    }

    if (S.size() > head) {
        Eigen::Index tail_start = std::max<Eigen::Index>(head, S.size() - 5);
        std::cout << "  ...\n";
        std::cout << "奇异值谱 (末 " << (S.size() - tail_start) << "):\n";
        for (Eigen::Index i = tail_start; i < S.size(); ++i) {
            std::cout << "  sigma[" << i << "] = " << std::setw(14) << S(i);
            if (S(i) <= threshold)
                std::cout << "  <-- 秩亏方向";
            std::cout << "\n";
        }
    }

    std::cout << "\n";
    return 0;
}
