cmake_minimum_required(VERSION 3.16)
project(brainco_identification LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_library(brainco_id_core
  src/data_loader.cpp
  src/algorithms.cpp
  src/dynamics_params.cpp
  src/revoarm_new_regressor.cpp
)
target_include_directories(brainco_id_core PUBLIC include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(brainco_id_core PUBLIC Eigen3::Eigen)

add_executable(identify main.cpp)
target_compile_definitions(identify PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(identify PRIVATE brainco_id_core)
