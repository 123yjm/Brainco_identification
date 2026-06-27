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
  src/robot_utils.cpp
  src/base_parameter.cpp
)
target_include_directories(brainco_id_core PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${NLOPT_INCLUDE_DIRS})
target_link_libraries(brainco_id_core PUBLIC Eigen3::Eigen ${YAML_CPP_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(identify_inertia src/app/main_solve_inertia.cpp)
target_compile_definitions(identify_inertia PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(identify_inertia PRIVATE brainco_id_core)

# ---- 重力分离 ----
add_executable(remove_gravity src/app/main_remove_gravity.cpp)
target_compile_definitions(remove_gravity PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(remove_gravity PRIVATE brainco_id_core)

# ---- 摩擦力辨识 ----
add_executable(identify_friction src/app/main_solve_friction.cpp)
target_compile_definitions(identify_friction PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(identify_friction PRIVATE brainco_id_core)

# ---- 激励轨迹优化（独立可执行文件）----
add_executable(get_excite_traj src/app/main_get_excite_traj.cpp)
target_compile_definitions(get_excite_traj PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(get_excite_traj PRIVATE brainco_id_core)

# ---- 摩擦力辨识轨迹生成 ----
add_executable(get_friction_traj src/app/main_get_friction_traj.cpp)
target_compile_definitions(get_friction_traj PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(get_friction_traj PRIVATE brainco_id_core)

# ---- 观测矩阵秩分析 ----
add_executable(calc_rank src/app/main_calc_rank.cpp)
target_compile_definitions(calc_rank PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(calc_rank PRIVATE brainco_id_core)

# ---- 最小惯性参数集辨识 (QR column-pivoting 分解) ----
add_executable(identify_base_inertia src/app/main_solve_base_inertia.cpp)
target_compile_definitions(identify_base_inertia PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(identify_base_inertia PRIVATE brainco_id_core)

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
  src/robot_utils.cpp
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
