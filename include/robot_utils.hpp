#pragma once

#include <filesystem>
#include <string>

/// 机器人目录工具函数 — 从 robot 根目录自动推导所有配置/数据/输出路径。
namespace robot_utils {

/// 从 robot_dir 提取机器人名（取最后一级目录名）。
/// 例如: "robots/revoarm_right" → "revoarm_right"
std::string robotNameFromDir(const std::string& robot_dir);

/// 拼接配置文件路径: <robot_dir>/config/<filename>
std::string configPath(const std::string& robot_dir, const std::string& filename);

/// 拼接数据文件路径: <robot_dir>/data/<filename>
std::string dataPath(const std::string& robot_dir, const std::string& filename);

/// 拼接结果文件路径: <robot_dir>/result/<filename>
std::string resultPath(const std::string& robot_dir, const std::string& filename);

/// 在目录下查找第一个匹配 glob 的文件，返回完整路径；找不到返回空串。
std::string findFirstFile(const std::string& directory, const std::string& glob);

/// 解析 -r/--robot 参数为完整机器人目录路径。
/// 若 arg 不含 '/'，自动前缀 "robots/"。
/// 例如: "revoarm_right" → "robots/revoarm_right"
std::string resolveRobotDir(const std::string& arg);

/// 将相对路径转为绝对路径（相对于 base_dir）。
inline std::string resolvePath(const std::string& path, const std::string& base_dir) {
    std::filesystem::path p(path);
    if (p.is_absolute()) return path;
    return (std::filesystem::path(base_dir) / p).string();
}

}  // namespace robot_utils
