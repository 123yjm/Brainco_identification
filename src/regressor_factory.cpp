#include "regressor_factory.hpp"
#include "serial_arm_regressor.hpp"

#include <stdexcept>
#include <string>

namespace robot_dynamics {

std::unique_ptr<IDynamicsRegressor>
RegressorFactory::create(const std::string &robot_name,
                         const std::string &yaml_path) {
  if (robot_name == "serial_arm") {
    return std::make_unique<SerialArmRegressor>(yaml_path);
  }

  throw std::runtime_error("未识别的 robot 类型: \"" + robot_name +
                           "\"。当前支持: serial_arm");
}

} // namespace robot_dynamics
