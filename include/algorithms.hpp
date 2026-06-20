#ifndef IDENTIFICATION_ALGORITHMS_HPP_
#define IDENTIFICATION_ALGORITHMS_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace identification {

/**
 * @brief Base class for identification algorithms
 */
class IdentificationAlgorithm {
public:
  virtual ~IdentificationAlgorithm() = default;

  virtual Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                                const Eigen::VectorXd &Tau_meas) = 0;

  void setUseRegularization(bool use) { use_regularization_ = use; }
  bool useRegularization() const { return use_regularization_; }

protected:
  bool use_regularization_{true};
};

class OLS : public IdentificationAlgorithm {
public:
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;
};

class WLS : public IdentificationAlgorithm {
public:
  explicit WLS(int dof);
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;
private:
  int dof_;
};

class IRLS : public IdentificationAlgorithm {
public:
  explicit IRLS(int max_iter = 50, double tol = 1e-4);
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;
private:
  int max_iter_;
  double tol_;
};

class TLS : public IdentificationAlgorithm {
public:
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;
};

class EKF : public IdentificationAlgorithm {
public:
  explicit EKF(int n_params);
  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;
private:
  int n_params_;
};

class NonlinearFrictionLM : public IdentificationAlgorithm {
public:
  explicit NonlinearFrictionLM(int dof, int multi_start = 4);

  Eigen::VectorXd solve(const Eigen::MatrixXd &W,
                        const Eigen::VectorXd &Tau_meas) override;

  void setVelocityData(const Eigen::MatrixXd &qd_meas);

  static std::size_t frictionParameterCount(int dof) {
    return static_cast<std::size_t>(6 * dof);
  }

  static Eigen::VectorXd predictTorques(const Eigen::MatrixXd &W_base,
                                        const Eigen::MatrixXd &qd_meas,
                                        const Eigen::VectorXd &params,
                                        int dof);

private:
  int dof_;
  int multi_start_;
  Eigen::MatrixXd qd_meas_;
};

// Factory
std::unique_ptr<IdentificationAlgorithm>
createAlgorithm(const std::string &type, int dof = 7);

} // namespace identification

#endif // IDENTIFICATION_ALGORITHMS_HPP_
