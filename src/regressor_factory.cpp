#include "regressor_factory.hpp"
#include "revoarm_new_regressor.hpp"

#include <stdexcept>
#include <string>

namespace robot_dynamics {

std::unique_ptr<IDynamicsRegressor>
RegressorFactory::create(const std::string &robot_name,
                         const std::string &yaml_path) {
  if (robot_name == "revoarm_new") {
    return std::make_unique<RevoarmNewRegressor>(yaml_path);
  }

  throw std::runtime_error("未识别的 robot 类型: \"" + robot_name +
                           "\"。当前支持: revoarm_new");
}

} // namespace robot_dynamics
