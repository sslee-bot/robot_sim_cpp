cmake_minimum_required(VERSION 3.0.0)
project(RobotSimCpp VERSION 1.0.0)

find_package(Eigen3 REQUIRED)
include(CTest)
enable_testing()

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(01_matrix_basic src/01_matrix_basic.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
