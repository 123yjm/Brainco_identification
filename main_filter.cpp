/**
 * @file main_filter.cpp
 * @brief 巴特沃斯低通滤波数据预处理入口（独立可执行文件）
 *
 * 读取 43 列原始激励轨迹采样 .txt 文件，设计数字 Butterworth 低通滤波器，
 * 进行零相位滤波 + 中心差分加速度计算，输出 29 列 CSV 文件。
 *
 * 用法:
 *   ./filter_data [--config <yaml>] [--input <txt>] [--output <csv>]
 *                 [--passband <Hz>] [--stopband <Hz>] [--help]
 *
 * 默认配置: config/butterworth_filter.yaml
 */

#include "butterworth_filter.hpp"
#include "csv_io.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef PROJECT_ROOT_DIR
#define PROJECT_ROOT_DIR "."
#endif

using signal_processing::FilterConfig;

namespace {

// ---------------------------------------------------------------------------
// 路径解析
// ---------------------------------------------------------------------------
inline std::string resolvePath(const std::string& path) {
    std::filesystem::path p(path);
    if (p.is_absolute()) return path;
    return (std::filesystem::path(PROJECT_ROOT_DIR) / p).string();
}

// ---------------------------------------------------------------------------
// 运行时配置（YAML + 命令行覆盖）
// ---------------------------------------------------------------------------
struct RuntimeOptions {
    std::string config_file = "config/butterworth_filter.yaml";
    FilterConfig filter_cfg;
};

// ---------------------------------------------------------------------------
// 打印帮助信息
// ---------------------------------------------------------------------------
void printHelp(const char* prog_name) {
    std::cout << "巴特沃斯滤波器 — 激励轨迹数据预处理\n"
              << "用法: " << prog_name << " [选项]\n\n"
              << "选项:\n"
              << "  --config <yaml>    YAML 配置文件 (默认: config/butterworth_filter.yaml)\n"
              << "  --input <txt>      覆盖输入 .txt 文件路径\n"
              << "  --output <csv>     覆盖输出 .csv 文件路径\n"
              << "  --passband <Hz>    覆盖通带频率\n"
              << "  --stopband <Hz>    覆盖阻带频率\n"
              << "  --help             打印本帮助信息并退出\n"
              << std::endl;
}

// ---------------------------------------------------------------------------
// 加载配置并解析命令行参数
// ---------------------------------------------------------------------------
RuntimeOptions parseOptions(int argc, char* argv[]) {
    RuntimeOptions opts;

    // 第一遍: 解析 --config
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            opts.config_file = argv[++i];
        } else if (arg == "--help") {
            printHelp(argv[0]);
            std::exit(0);
        }
    }

    // 加载 YAML 配置
    std::string config_path = resolvePath(opts.config_file);
    opts.filter_cfg = signal_processing::loadFilterConfig(config_path, PROJECT_ROOT_DIR);

    // 第二遍: 命令行覆盖
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            ++i;  // skip value
        } else if (arg == "--input" && i + 1 < argc) {
            opts.filter_cfg.input_txt = resolvePath(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            opts.filter_cfg.output_csv = resolvePath(argv[++i]);
        } else if (arg == "--passband" && i + 1 < argc) {
            opts.filter_cfg.passband_hz = std::stod(argv[++i]);
        } else if (arg == "--stopband" && i + 1 < argc) {
            opts.filter_cfg.stopband_hz = std::stod(argv[++i]);
        } else if (arg == "--help") {
            printHelp(argv[0]);
            std::exit(0);
        }
    }

    return opts;
}

}  // anonymous namespace

// ============================================================================
int main(int argc, char* argv[]) {
    RuntimeOptions opts = parseOptions(argc, argv);
    const FilterConfig& cfg = opts.filter_cfg;

    // ---- 1. 打印配置 --------------------------------------------------------
    std::cout << "═══════════════════════════════════════════\n"
              << "  Butterworth 滤波器 — 数据预处理\n"
              << "═══════════════════════════════════════════\n"
              << "配置文件:     " << opts.config_file << "\n"
              << "输入文件:     " << cfg.input_txt << "\n"
              << "输出文件:     " << cfg.output_csv << "\n"
              << "采样频率:     " << cfg.fs << " Hz\n"
              << "通带频率:     " << cfg.passband_hz << " Hz\n"
              << "阻带频率:     " << cfg.stopband_hz << " Hz\n"
              << "通带波纹:     " << cfg.rp_db << " dB\n"
              << "阻带衰减:     " << cfg.rs_db << " dB\n"
              << std::endl;

    // ---- 2. 读取原始数据 ----------------------------------------------------
    std::cout << "读取原始数据..." << std::endl;
    signal_processing::RawMeasurementData raw;
    try {
        raw = signal_processing::readRawTxt(cfg.input_txt);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "  样本数: " << raw.n_samples
              << ", 自由度: " << raw.n_dof << std::endl;

    // ---- 3. 设计滤波器 ------------------------------------------------------
    double fs = cfg.fs;
    double Wp = cfg.passband_hz / (fs / 2.0);
    double Ws = cfg.stopband_hz / (fs / 2.0);

    std::cout << "\n设计 Butterworth 低通滤波器...\n"
              << "  归一化通带: " << Wp << "\n"
              << "  归一化阻带: " << Ws << std::endl;

    signal_processing::ButterworthFilterDesign design;
    try {
        design = signal_processing::designButterworthLowpass(
            Wp, Ws, cfg.rp_db, cfg.rs_db, fs);
    } catch (const std::exception& e) {
        std::cerr << "滤波器设计失败: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "  滤波器阶数: " << design.order << "\n"
              << "  归一化截止频率: " << design.cutoff_normalized << "\n"
              << "  分子系数 b: [";
    for (int i = 0; i <= design.order; ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << design.b(i);
    }
    std::cout << "]\n  分母系数 a: [";
    for (int i = 0; i <= design.order; ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << design.a(i);
    }
    std::cout << "]" << std::endl;

    // ---- 4. 滤波 ------------------------------------------------------------
    double Ts = 1.0 / fs;
    std::cout << "\n应用零相位滤波..." << std::endl;

    signal_processing::FilteredOutputData filtered;
    filtered.n_samples = raw.n_samples;
    filtered.n_dof = raw.n_dof;

    // 相对时间
    filtered.time.resize(raw.n_samples);
    double t0 = raw.t_abs[0];
    for (int i = 0; i < raw.n_samples; ++i) {
        filtered.time[i] = raw.t_abs[i] - t0;
    }

    // q_filtered = q (不滤波，直接复制)
    filtered.q_filtered = raw.q;
    std::cout << "  q:           不滤波，直接复制" << std::endl;

    // q_dot_filtered = filtfilt(q_dot)
    filtered.q_dot_filtered = signal_processing::filtfilt(
        design.b, design.a, raw.q_dot);
    std::cout << "  q_dot:       零相位滤波完成" << std::endl;

    // q_ddot_filtered = centralDifference(q_dot_filtered)
    filtered.q_ddot_filtered = signal_processing::centralDifference(
        filtered.q_dot_filtered, Ts);
    std::cout << "  q_ddot:      中心差分计算完成 (Ts = " << Ts << " s)" << std::endl;

    // tau_filtered = filtfilt(motor_current)
    filtered.tau_filtered = signal_processing::filtfilt(
        design.b, design.a, raw.motor_current);
    std::cout << "  tau:         零相位滤波完成" << std::endl;

    // ---- 5. 写入 CSV --------------------------------------------------------
    std::cout << "\n写入输出 CSV..." << std::endl;
    try {
        signal_processing::writeFilteredCsv(cfg.output_csv, filtered);
    } catch (const std::exception& e) {
        std::cerr << "写入 CSV 失败: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "  → " << cfg.output_csv << std::endl;

    // ---- 6. 总结 ------------------------------------------------------------
    std::cout << "\n═══════════════════════════════════════════\n"
              << "  处理完成\n"
              << "  样本数: " << filtered.n_samples << "\n"
              << "  自由度: " << filtered.n_dof << "\n"
              << "  滤波器阶数: " << design.order << "\n"
              << "  输出: " << cfg.output_csv << "\n"
              << "═══════════════════════════════════════════\n"
              << std::endl;

    return 0;
}
