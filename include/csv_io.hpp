#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace signal_processing {

/// Filter parameters loaded from butterworth_filter.yaml.
/// I/O paths are auto-derived from the robot directory by entry points.
struct FilterConfig {
    bool turn_on_filter = true;
    double fs = 100.0;
    double passband_hz = 5.0;
    double stopband_hz = 10.0;
    double rp_db = 3.0;
    double rs_db = 30.0;
};

/// Raw data parsed from the 43-column comma-separated .txt file.
struct RawMeasurementData {
    std::vector<double> t_abs;       ///< Col 1:  absolute Unix timestamps [s]
    Eigen::MatrixXd q;               ///< Cols 2-8:   joint positions (N x 7) [rad]
    Eigen::MatrixXd q_dot;           ///< Cols 9-15:  joint velocities (N x 7) [rad/s]
    Eigen::MatrixXd motor_current;   ///< Cols 16-22: motor currents (N x 7) [A]
    Eigen::MatrixXd qd;              ///< Cols 23-29: desired positions (N x 7) [rad]
    Eigen::MatrixXd qd_dot;          ///< Cols 30-36: desired velocities (N x 7) [rad/s]
    Eigen::MatrixXd qd_ddot;         ///< Cols 37-43: desired accelerations (N x 7) [rad/s²]
    int n_samples = 0;
    int n_dof = 7;
};

/// Filtered output data matching the 29-column CSV format.
struct FilteredOutputData {
    std::vector<double> time;          ///< Relative time from 0 [s]
    Eigen::MatrixXd q_filtered;        ///< Unfiltered q copy (N x 7) [rad]
    Eigen::MatrixXd q_dot_filtered;    ///< filtfilt on q_dot (N x 7) [rad/s]
    Eigen::MatrixXd q_ddot_filtered;   ///< Central diff on q_dot_filtered (N x 7) [rad/s²]
    Eigen::MatrixXd tau_filtered;      ///< filtfilt on motor_current (N x 7)

    // ---- 滤波前原始数据（用于 CSV 对比滤波效果）----
    Eigen::MatrixXd q_dot_raw;         ///< Raw q_dot before filtering (N x 7)
    Eigen::MatrixXd q_ddot_raw;        ///< Central diff on raw q_dot (N x 7)
    Eigen::MatrixXd tau_raw;           ///< Raw motor_current before filtering (N x 7)

    int n_samples = 0;
    int n_dof = 7;
};

/// Load filter parameters from a YAML file (uses yaml-cpp).
///
/// @param yaml_path  Path to the YAML configuration file
/// @return           Populated FilterConfig
FilterConfig loadFilterConfig(const std::string& yaml_path);

/// Read a 43-column headerless comma-separated .txt file.
///
/// Expected columns per row:
///   t_abs, q[0..6], q_dot[0..6], motor_current[0..6],
///   qd[0..6], qd_dot[0..6], qd_ddot[0..6]
///
/// @param filepath  Absolute or relative path to the input file
/// @return          Parsed RawMeasurementData
RawMeasurementData readRawTxt(const std::string& filepath);

/// Write filtered data as a 29-column CSV with header.
///
/// Column order:
///   time, q0..q6, qd0..qd6, qdd0..qdd6, tau0..tau6
///
/// @param filepath  Output CSV path
/// @param data      Filtered data to write
void writeFilteredCsv(const std::string& filepath,
                      const FilteredOutputData& data);

}  // namespace signal_processing
