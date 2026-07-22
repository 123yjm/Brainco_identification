#include "data_loader.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

ExperimentData DataLoader::loadCSV(const std::string &filename,
                                   std::size_t n_dof) {
  ExperimentData data;
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + filename);
  }

  std::string line;
  // Read header to detect format
  std::getline(file, line);

  // Detect if qdd columns exist by checking header or column count
  // Format with qdd: time, q0..qN, qd0..qdN, qdd0..qddN, tau0..tauN
  // Format without qdd: time, q0..qN, qd0..qdN, tau0..tauN
  const std::size_t cols_without_qdd = 1 + 3 * n_dof;   // 22: time,q,qd,tau
  const std::size_t cols_with_qdd    = 1 + 4 * n_dof;   // 29: time,q,qd,qdd,tau
  const std::size_t cols_with_ref    = 1 + 6 * n_dof;   // 43: time,q,dq,tau,q_ref,dq_ref,tau_ref
  const std::size_t cols_with_raw    = 1 + 8 * n_dof;   // 57: filtered+raw combined

  std::vector<double> time_vec;
  std::vector<std::vector<double>> q_vec, qd_vec, qdd_vec, tau_vec, tau_raw_vec;
  bool has_qdd = false;
  bool is_ref_format = false;   // 43-column raw format with q/dq/tau ref signals
  bool format_detected = false;
  std::size_t expected_cols = 0;

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    std::vector<double> row;

    while (std::getline(ss, token, ',')) {
      row.push_back(std::stod(token));
    }

    // Detect format on first data row
    if (!format_detected) {
      if (row.size() == cols_without_qdd) {
        has_qdd = false;
        expected_cols = cols_without_qdd;
      } else if (row.size() == cols_with_qdd) {
        has_qdd = true;
        expected_cols = cols_with_qdd;
      } else if (row.size() == cols_with_ref) {
        // 43-column raw format: [time, q_actual, dq_actual, tau_actual, q_ref, dq_ref, tau_ref]
        has_qdd = false;
        is_ref_format = true;
        expected_cols = cols_with_ref;
      } else if (row.size() >= cols_with_raw) {
        has_qdd = true;
        expected_cols = row.size();
      } else {
        throw std::runtime_error("CSV 列数与配置的机械臂自由度不匹配: " +
                                 std::to_string(row.size()) + " 列, 期望 " +
                                 std::to_string(cols_without_qdd) + "、" +
                                 std::to_string(cols_with_qdd) + "、" +
                                 std::to_string(cols_with_ref) + " 或 " +
                                 std::to_string(cols_with_raw) + " 列");
      }
      format_detected = true;
    }

    // Validate row size consistency
    if (row.size() != expected_cols) {
      throw std::runtime_error("CSV 数据行列数不一致: 实际 " +
                               std::to_string(row.size()) + " 列, 期望 " +
                               std::to_string(expected_cols) + " 列");
    }

    time_vec.push_back(row[0]);

    std::vector<double> q_row, qd_row, qdd_row, tau_row;

    if (is_ref_format) {
      // 43-col: [time, q_actual(7), dq_actual(7), tau_actual(7), q_ref(7), dq_ref(7), tau_ref(7)]
      for (std::size_t i = 0; i < n_dof; ++i)
        q_row.push_back(row[1 + i]);
      for (std::size_t i = 0; i < n_dof; ++i)
        qd_row.push_back(row[1 + n_dof + i]);
      for (std::size_t i = 0; i < n_dof; ++i)
        tau_row.push_back(row[1 + 2 * n_dof + i]);
      // q_ref = cols 1+3*n_dof..1+4*n_dof, dq_ref, tau_ref — discarded
    } else {
      for (std::size_t i = 0; i < n_dof; ++i)
        q_row.push_back(row[1 + i]);
      for (std::size_t i = 0; i < n_dof; ++i)
        qd_row.push_back(row[1 + n_dof + i]);

      if (has_qdd) {
        for (std::size_t i = 0; i < n_dof; ++i)
          qdd_row.push_back(row[1 + 2 * n_dof + i]);
        for (std::size_t i = 0; i < n_dof; ++i)
          tau_row.push_back(row[1 + 3 * n_dof + i]);
      } else {
        for (std::size_t i = 0; i < n_dof; ++i)
          tau_row.push_back(row[1 + 2 * n_dof + i]);
      }

      // Read tau_raw if present (>= 57 columns: raw comparison data)
      // tau_raw is at columns 1+7*n_dof .. 1+8*n_dof-1
      if (row.size() >= cols_with_raw) {
        std::vector<double> tau_raw_row;
        for (std::size_t i = 0; i < n_dof; ++i)
          tau_raw_row.push_back(row[1 + 7 * n_dof + i]);
        tau_raw_vec.push_back(tau_raw_row);
      }
    }

    q_vec.push_back(q_row);
    qd_vec.push_back(qd_row);
    if (has_qdd) {
      qdd_vec.push_back(qdd_row);
    }
    tau_vec.push_back(tau_row);
  }

  data.n_samples = time_vec.size();
  data.n_dof = n_dof;
  data.time = time_vec;

  data.q.resize(data.n_samples, n_dof);
  data.qd.resize(data.n_samples, n_dof);
  data.tau.resize(data.n_samples, n_dof);

  for (std::size_t i = 0; i < data.n_samples; ++i) {
    for (std::size_t j = 0; j < n_dof; ++j) {
      data.q(i, j) = q_vec[i][j];
      data.qd(i, j) = qd_vec[i][j];
      data.tau(i, j) = tau_vec[i][j];
    }
  }

  // Load qdd if present, otherwise leave empty for preprocess to compute
  if (has_qdd) {
    data.qdd.resize(data.n_samples, n_dof);
    for (std::size_t i = 0; i < data.n_samples; ++i) {
      for (std::size_t j = 0; j < n_dof; ++j) {
        data.qdd(i, j) = qdd_vec[i][j];
      }
    }
    std::cout << "Loaded " << data.n_samples << " samples from " << filename
              << " (with qdd from physics engine)" << std::endl;
  } else {
    std::cout << "Loaded " << data.n_samples << " samples from " << filename
              << " (qdd will be computed via numerical differentiation)"
              << std::endl;
  }

  // Load tau_raw if present
  if (!tau_raw_vec.empty()) {
    data.tau_raw.resize(data.n_samples, n_dof);
    for (std::size_t i = 0; i < data.n_samples; ++i) {
      for (std::size_t j = 0; j < n_dof; ++j) {
        data.tau_raw(i, j) = tau_raw_vec[i][j];
      }
    }
  }

  return data;
}
