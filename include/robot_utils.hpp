#pragma once

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

}  // namespace robot_utils
