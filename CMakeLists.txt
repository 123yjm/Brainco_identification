cmake_minimum_required(VERSION 3.16)
project(brainco_identification LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH /opt/ros/humble)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)

add_library(brainco_id_core
  src/data_loader.cpp
  src/algorithms.cpp
  src/body2inertial.cpp
  src/revoarm_new_regressor.cpp
  src/robot_config_loader.cpp
)
target_include_directories(brainco_id_core PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(brainco_id_core PUBLIC Eigen3::Eigen ${YAML_CPP_LIBRARIES} pinocchio::pinocchio)

add_executable(identify main.cpp)
target_compile_definitions(identify PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(identify PRIVATE brainco_id_core)
