/**
 * @file main_filter.cpp
 * @brief 巴特沃斯低通滤波数据预处理入口
 *
 * 用法: ./filter_data --robot <robot_dir> [--passband <Hz>] [--stopband <Hz>] [--help]
 */

#include "butterworth_filter.hpp"
#include "csv_io.hpp"
#include "robot_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

void printHelp(const char* prog) {
    std::cout << "巴特沃斯滤波器 — 数据预处理\n"
              << "用法: " << prog << " --robot <robot_dir> [选项]\n\n"
              << "选项:\n"
              << "  --robot <dir>       机器人目录 (如 robots/revoarm_right)\n"
              << "  --passband <Hz>     覆盖通带频率\n"
              << "  --stopband <Hz>     覆盖阻带频率\n"
              << "  --help              打印帮助信息\n";
}

}  // anonymous namespace

int main(int argc, char* argv[]) {
    // ---- 解析 --help / --robot --------------------------------------------
    std::string robot_dir;
    double passband_override = -1, stopband_override = -1;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") { printHelp(argv[0]); return 0; }
        else if (arg == "--robot" && i + 1 < argc) robot_dir = argv[++i];
        else if (arg == "--passband" && i + 1 < argc) passband_override = std::stod(argv[++i]);
        else if (arg == "--stopband" && i + 1 < argc) stopband_override = std::stod(argv[++i]);
    }

    if (robot_dir.empty()) {
        std::cerr << "错误: 需要 --robot <dir>\n";
        printHelp(argv[0]);
        return 1;
    }

    std::string robot_name = robot_utils::robotNameFromDir(robot_dir);

    // ---- 1. 加载配置 ------------------------------------------------------
    std::string filter_yaml = robot_utils::configPath(robot_dir, "butterworth_filter.yaml");
    signal_processing::FilterConfig cfg = signal_processing::loadFilterConfig(filter_yaml);
    if (passband_override > 0) cfg.passband_hz = passband_override;
    if (stopband_override > 0) cfg.stopband_hz = stopband_override;

    std::string input_txt = robot_utils::findFirstFile(
        robot_utils::dataPath(robot_dir, ""), "*.txt");
    if (input_txt.empty()) {
        std::cerr << "错误: 在 " << robot_utils::dataPath(robot_dir, "")
                  << " 下未找到 .txt 文件\n";
        return 1;
    }

    std::string output_csv = robot_utils::resultPath(robot_dir,
        robot_name + "_filtered_data.csv");

    // ---- 2. 打印配置 ------------------------------------------------------
    std::cout << "═══════════════════════════════════════════\n"
              << "  Butterworth 滤波器 — 数据预处理\n"
              << "═══════════════════════════════════════════\n"
              << "机器人:       " << robot_name << "\n"
              << "输入:         " << input_txt << "\n"
              << "输出:         " << output_csv << "\n"
              << "通带:         " << cfg.passband_hz << " Hz\n"
              << "阻带:         " << cfg.stopband_hz << " Hz\n"
              << std::endl;

    // ---- 3. 读取原始数据 --------------------------------------------------
    signal_processing::RawMeasurementData raw;
    try {
        raw = signal_processing::readRawTxt(input_txt);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "样本数: " << raw.n_samples << ", DOF: " << raw.n_dof << std::endl;

    // ---- 4. 设计滤波器 ----------------------------------------------------
    double fs = cfg.fs;
    double Wp = cfg.passband_hz / (fs / 2.0);
    double Ws = cfg.stopband_hz / (fs / 2.0);

    auto design = signal_processing::designButterworthLowpass(
        Wp, Ws, cfg.rp_db, cfg.rs_db, fs);
    std::cout << "滤波器阶数: " << design.order
              << ", 截止频率: " << design.cutoff_normalized << std::endl;

    // ---- 5. 滤波 ----------------------------------------------------------
    double Ts = 1.0 / fs;
    signal_processing::FilteredOutputData filtered;
    filtered.n_samples = raw.n_samples;
    filtered.n_dof = raw.n_dof;

    filtered.time.resize(raw.n_samples);
    double t0 = raw.t_abs[0];
    for (int i = 0; i < raw.n_samples; ++i) filtered.time[i] = raw.t_abs[i] - t0;

    filtered.q_filtered      = raw.q;
    filtered.q_dot_filtered  = signal_processing::filtfilt(design.b, design.a, raw.q_dot);
    filtered.q_ddot_filtered = signal_processing::centralDifference(filtered.q_dot_filtered, Ts);
    filtered.tau_filtered    = signal_processing::filtfilt(design.b, design.a, raw.motor_current);

    // ---- 6. 写入 CSV ------------------------------------------------------
    std::filesystem::create_directories(robot_utils::resultPath(robot_dir, ""));
    signal_processing::writeFilteredCsv(output_csv, filtered);

    std::cout << "输出: " << output_csv << "\n完成\n" << std::endl;
    return 0;
}
