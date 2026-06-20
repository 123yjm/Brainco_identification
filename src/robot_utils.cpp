#include "robot_utils.hpp"

#include <filesystem>
#include <string>

namespace robot_utils {

std::string robotNameFromDir(const std::string& robot_dir) {
    std::filesystem::path p(robot_dir);
    return p.filename().string();
}

std::string configPath(const std::string& robot_dir, const std::string& filename) {
    std::filesystem::path p(robot_dir);
    p /= "config";
    p /= filename;
    return p.string();
}

std::string dataPath(const std::string& robot_dir, const std::string& filename) {
    std::filesystem::path p(robot_dir);
    p /= "data";
    p /= filename;
    return p.string();
}

std::string resultPath(const std::string& robot_dir, const std::string& filename) {
    std::filesystem::path p(robot_dir);
    p /= "result";
    p /= filename;
    return p.string();
}

std::string findFirstFile(const std::string& directory, const std::string& glob) {
    std::filesystem::path dir(directory);
    if (!std::filesystem::is_directory(dir)) return {};

    // 简单 glob: 仅支持 "*.ext" 形式
    std::string ext;
    if (glob.size() >= 2 && glob[0] == '*' && glob[1] == '.') {
        ext = glob.substr(1);  // ".ext"
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        auto path = entry.path();
        if (!ext.empty() && path.extension().string() != ext) continue;
        return path.string();  // 返回第一个匹配
    }
    return {};
}

}  // namespace robot_utils
