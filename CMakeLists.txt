cmake_minimum_required(VERSION 3.16)
project(brainco_identification LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# NLopt (pkg-config)
find_package(PkgConfig)
pkg_check_modules(NLOPT REQUIRED nlopt_cxx)

add_library(brainco_id_core
  src/data_loader.cpp
  src/algorithms.cpp
  src/body2inertial.cpp
  src/serial_arm_regressor.cpp
  src/robot_config_loader.cpp
  src/regressor_factory.cpp
  src/trajectory_optimizer.cpp
)
target_include_directories(brainco_id_core PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${NLOPT_INCLUDE_DIRS})
target_link_libraries(brainco_id_core PUBLIC Eigen3::Eigen ${YAML_CPP_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(identify src/app/main_solve.cpp)
target_compile_definitions(identify PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(identify PRIVATE brainco_id_core)

# ---- 激励轨迹优化（独立可执行文件）----
add_executable(get_traj src/app/main_traj.cpp)
target_compile_definitions(get_traj PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(get_traj PRIVATE brainco_id_core)

# ---- Qt 上位机（独立可执行文件，与辨识核心解耦）----
find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

add_executable(identify_gui
  src/qt/main.cpp
  src/qt/main_window.cpp
)
target_include_directories(identify_gui PRIVATE include ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(identify_gui PRIVATE Qt5::Widgets ${YAML_CPP_LIBRARIES})

# ---- 巴特沃斯滤波数据预处理（独立静态库，与辨识核心解耦）----
add_library(brainco_id_filter
  src/butterworth_filter.cpp
  src/csv_io.cpp
)
target_include_directories(brainco_id_filter PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(brainco_id_filter PUBLIC Eigen3::Eigen ${YAML_CPP_LIBRARIES})

# ---- 巴特沃斯滤波 CLI 入口（独立可执行文件）----
add_executable(filter_data src/app/main_filter.cpp)
target_compile_definitions(filter_data PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(filter_data PRIVATE brainco_id_filter)

# ---- 滤波 + 辨识一站式入口（无中间 CSV 落盘）----
add_executable(filter_and_solve src/app/main_filter_and_solve.cpp)
target_compile_definitions(filter_and_solve PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(filter_and_solve PRIVATE brainco_id_filter brainco_id_core)
