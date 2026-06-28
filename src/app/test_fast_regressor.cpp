/**
 * 全轨迹 W 矩阵对比: fast vs original regressor
 */
#include "base_parameter.hpp"
#include "fast_regressor.hpp"
#include "serial_arm_regressor.hpp"
#include "regressor_factory.hpp"
#include "robot_utils.hpp"
#include "data_loader.hpp"
#include <iostream>
#include <iomanip>

int main() {
    std::string robot_dir = robot_utils::resolveRobotDir("revoarm_right");
    robot_dir = robot_utils::resolvePath(robot_dir, PROJECT_ROOT_DIR);
    std::string kinematic_yaml = robot_utils::configPath(robot_dir, "kinematic_params.yaml");

    auto reg = robot_dynamics::RegressorFactory::create("serial_arm", kinematic_yaml);
    auto* sar = dynamic_cast<robot_dynamics::SerialArmRegressor*>(reg.get());
    robot_dynamics::initFastRegressor(*sar);

    auto flags = robot_dynamics::ParamFlags::NONE;

    // Load trajectory CSV
    std::string csv_path = robot_utils::dataBaseInertiaPath(robot_dir, "revoarm_right_filtered_data.csv");
    auto data = DataLoader::loadCSV(csv_path, reg->nDof());
    std::cout << "Loaded " << data.n_samples << " samples\n";

    int n_dof = reg->nDof();
    int n_params = reg->numParameters(flags);
    int K = data.n_samples;

    // Build W with original regressor
    Eigen::MatrixXd W_orig(K * n_dof, n_params);
    for (int k = 0; k < K; ++k) {
        Eigen::VectorXd q = data.q.row(k).transpose();
        Eigen::VectorXd qd = data.qd.row(k).transpose();
        Eigen::VectorXd qdd = data.qdd.row(k).transpose();
        auto Y = sar->computeRegressorMatrix(q, qd, qdd, flags);
        W_orig.block(k * n_dof, 0, n_dof, n_params) = Y;
    }

    // Build W with fast regressor
    Eigen::MatrixXd W_fast(K * n_dof, n_params);
    for (int k = 0; k < K; ++k) {
        Eigen::VectorXd q = data.q.row(k).transpose();
        Eigen::VectorXd qd = data.qd.row(k).transpose();
        Eigen::VectorXd qdd = data.qdd.row(k).transpose();
        auto Y = robot_dynamics::computeRegressorFast(q, qd, qdd, flags);
        W_fast.block(k * n_dof, 0, n_dof, n_params) = Y;
    }

    // Compare Y's per time step
    double max_Y_diff = 0.0;
    for (int k = 0; k < K; ++k) {
        Eigen::MatrixXd Y_orig = W_orig.block(k * n_dof, 0, n_dof, n_params);
        Eigen::MatrixXd Y_fast = W_fast.block(k * n_dof, 0, n_dof, n_params);
        double diff = (Y_orig - Y_fast).cwiseAbs().maxCoeff();
        if (diff > max_Y_diff) max_Y_diff = diff;
        if (diff > 1e-8 && k < 5)
            std::cout << "k=" << k << " max Y diff: " << std::scientific << diff << "\n";
    }
    std::cout << "Max Y diff across all " << K << " steps: " << max_Y_diff << "\n";

    // Compare full W
    double W_diff = (W_orig - W_fast).cwiseAbs().maxCoeff();
    std::cout << "Max W diff: " << W_diff << "\n\n";

    // QRCP on both
    auto d_orig = robot_dynamics::BaseParameterDecomposition::compute(W_orig, 1.0);
    auto d_fast = robot_dynamics::BaseParameterDecomposition::compute(W_fast, 1.0);
    std::cout << "QRCP orig: rank=" << d_orig.rank << " cond=" << d_orig.cond_R1 << "\n";
    std::cout << "QRCP fast: rank=" << d_fast.rank << " cond=" << d_fast.cond_R1 << "\n";

    // Eigenvalue-based cond
    {
        auto WtW = W_orig.transpose() * W_orig;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigs(WtW);
        auto &ev = eigs.eigenvalues();
        double lmax = ev(ev.size()-1);
        double threshold = 1e-12 * lmax;
        double lmin = lmax;
        for (int i = 0; i < ev.size(); ++i) { if (ev(i) > threshold) { lmin = ev(i); break; } }
        std::cout << "Eig cond orig: " << lmax/lmin << " (lmax=" << lmax << ", lmin=" << lmin << ")\n";
    }
    {
        auto WtW = W_fast.transpose() * W_fast;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigs(WtW);
        auto &ev = eigs.eigenvalues();
        double lmax = ev(ev.size()-1);
        double threshold = 1e-12 * lmax;
        double lmin = lmax;
        for (int i = 0; i < ev.size(); ++i) { if (ev(i) > threshold) { lmin = ev(i); break; } }
        std::cout << "Eig cond fast: " << lmax/lmin << " (lmax=" << lmax << ", lmin=" << lmin << ")\n";
    }

    return 0;
}
