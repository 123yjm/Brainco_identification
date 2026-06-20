cmake_minimum_required(VERSION 3.16)
project(brainco_identification LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(brainco_id_core
  src/data_loader.cpp
  src/algorithms.cpp
  src/body2inertial.cpp
  src/serial_arm_regressor.cpp
  src/robot_config_loader.cpp
  src/regressor_factory.cpp
)
target_include_directories(brainco_id_core PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(brainco_id_core PUBLIC Eigen3::Eigen ${YAML_CPP_LIBRARIES})

add_executable(identify main.cpp)
target_compile_definitions(identify PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(identify PRIVATE brainco_id_core)

# ---- Qt 上位机（独立可执行文件，与辨识核心解耦）----
find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

add_executable(identify_gui
  src/qt/main.cpp
  src/qt/main_window.cpp
)
target_include_directories(identify_gui PRIVATE include ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(identify_gui PRIVATE Qt5::Widgets ${YAML_CPP_LIBRARIES})
