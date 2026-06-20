#include "csv_io.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/// Split a string by delimiter.
std::vector<std::string> splitLine(const std::string& line, char delim) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, delim)) {
        tokens.push_back(token);
    }
    return tokens;
}

/// Resolve a possibly-relative path against base_dir.
std::string resolvePath(const std::string& path, const std::string& base_dir) {
    if (path.empty()) return path;
    if (path[0] == '/') return path;  // already absolute
    return base_dir + "/" + path;
}

}  // anonymous namespace

namespace signal_processing {

// ---------------------------------------------------------------------------
//  YAML Config Loader
// ---------------------------------------------------------------------------

FilterConfig loadFilterConfig(const std::string& yaml_path,
                              const std::string& base_dir) {
    FilterConfig cfg;
    YAML::Node root = YAML::LoadFile(yaml_path);

    // --- io section ---
    if (root["io"]) {
        auto io = root["io"];
        if (io["input_txt"]) {
            cfg.input_txt = resolvePath(io["input_txt"].as<std::string>(), base_dir);
        }
        if (io["output_csv"]) {
            cfg.output_csv = resolvePath(io["output_csv"].as<std::string>(), base_dir);
        }
    }

    // --- filter section ---
    if (root["filter"]) {
        auto flt = root["filter"];
        if (flt["sampling_frequency_hz"]) {
            cfg.fs = flt["sampling_frequency_hz"].as<double>();
        }
        if (flt["passband_hz"]) {
            cfg.passband_hz = flt["passband_hz"].as<double>();
        }
        if (flt["stopband_hz"]) {
            cfg.stopband_hz = flt["stopband_hz"].as<double>();
        }
        if (flt["passband_ripple_db"]) {
            cfg.rp_db = flt["passband_ripple_db"].as<double>();
        }
        if (flt["stopband_attenuation_db"]) {
            cfg.rs_db = flt["stopband_attenuation_db"].as<double>();
        }
    }

    return cfg;
}

// ---------------------------------------------------------------------------
//  Raw TXT Reader (43 columns, no header, comma-separated)
// ---------------------------------------------------------------------------

RawMeasurementData readRawTxt(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open input file: " + filepath);
    }

    constexpr int kExpectedCols = 43;
    constexpr int kDof = 7;

    // Temporary column-major accumulation vectors
    std::vector<double> t_abs_vec;
    std::vector<std::vector<double>> q_vec(kDof);
    std::vector<std::vector<double>> q_dot_vec(kDof);
    std::vector<std::vector<double>> motor_current_vec(kDof);
    std::vector<std::vector<double>> qd_vec(kDof);
    std::vector<std::vector<double>> qd_dot_vec(kDof);
    std::vector<std::vector<double>> qd_ddot_vec(kDof);

    std::string line;
    int line_no = 0;
    while (std::getline(file, line)) {
        ++line_no;
        if (line.empty()) continue;

        auto tokens = splitLine(line, ',');
        if (static_cast<int>(tokens.size()) != kExpectedCols) {
            throw std::runtime_error(
                "Line " + std::to_string(line_no) + ": expected " +
                std::to_string(kExpectedCols) + " columns, got " +
                std::to_string(tokens.size()));
        }

        // Col 0: t_abs
        t_abs_vec.push_back(std::stod(tokens[0]));

        // Cols 1-7: q
        for (int j = 0; j < kDof; ++j) {
            q_vec[j].push_back(std::stod(tokens[1 + j]));
        }
        // Cols 8-14: q_dot
        for (int j = 0; j < kDof; ++j) {
            q_dot_vec[j].push_back(std::stod(tokens[8 + j]));
        }
        // Cols 15-21: motor_current
        for (int j = 0; j < kDof; ++j) {
            motor_current_vec[j].push_back(std::stod(tokens[15 + j]));
        }
        // Cols 22-28: qd
        for (int j = 0; j < kDof; ++j) {
            qd_vec[j].push_back(std::stod(tokens[22 + j]));
        }
        // Cols 29-35: qd_dot
        for (int j = 0; j < kDof; ++j) {
            qd_dot_vec[j].push_back(std::stod(tokens[29 + j]));
        }
        // Cols 36-42: qd_ddot
        for (int j = 0; j < kDof; ++j) {
            qd_ddot_vec[j].push_back(std::stod(tokens[36 + j]));
        }
    }

    int N = static_cast<int>(t_abs_vec.size());
    if (N == 0) {
        throw std::runtime_error("Input file contains no data rows: " + filepath);
    }

    RawMeasurementData data;
    data.n_samples = N;
    data.n_dof = kDof;
    data.t_abs = std::move(t_abs_vec);

    // Allocate Eigen matrices and copy column-major → row-major
    auto fillMatrix = [N, kDof](const std::vector<std::vector<double>>& src) {
        Eigen::MatrixXd M(N, kDof);
        for (int j = 0; j < kDof; ++j) {
            for (int i = 0; i < N; ++i) {
                M(i, j) = src[j][i];
            }
        }
        return M;
    };

    data.q = fillMatrix(q_vec);
    data.q_dot = fillMatrix(q_dot_vec);
    data.motor_current = fillMatrix(motor_current_vec);
    data.qd = fillMatrix(qd_vec);
    data.qd_dot = fillMatrix(qd_dot_vec);
    data.qd_ddot = fillMatrix(qd_ddot_vec);

    return data;
}

// ---------------------------------------------------------------------------
//  Filtered CSV Writer (29 columns, with header, comma-separated)
// ---------------------------------------------------------------------------

void writeFilteredCsv(const std::string& filepath,
                      const FilteredOutputData& data) {
    // Ensure output directory exists
    std::filesystem::path p(filepath);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }

    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open output file: " + filepath);
    }

    file << std::fixed << std::setprecision(6);

    // --- Header ---
    file << "time";
    for (int j = 0; j < data.n_dof; ++j) file << ",q" << j;
    for (int j = 0; j < data.n_dof; ++j) file << ",qd" << j;
    for (int j = 0; j < data.n_dof; ++j) file << ",qdd" << j;
    for (int j = 0; j < data.n_dof; ++j) file << ",tau" << j;
    file << "\n";

    // --- Data rows ---
    for (int i = 0; i < data.n_samples; ++i) {
        file << data.time[i];
        for (int j = 0; j < data.n_dof; ++j) file << "," << data.q_filtered(i, j);
        for (int j = 0; j < data.n_dof; ++j) file << "," << data.q_dot_filtered(i, j);
        for (int j = 0; j < data.n_dof; ++j) file << "," << data.q_ddot_filtered(i, j);
        for (int j = 0; j < data.n_dof; ++j) file << "," << data.tau_filtered(i, j);
        file << "\n";
    }

    file.close();
}

}  // namespace signal_processing
