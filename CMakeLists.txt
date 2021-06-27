cmake_minimum_required(VERSION 3.0.0)
project(RobotSimCpp VERSION 1.0.0)

find_package(Eigen3 REQUIRED)
include(CTest)
enable_testing()

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(./include)

add_executable(01_matrix_basic
    src/01_matrix_basic.cpp)

add_executable(02_CARE
    src/class/algorithm/CARE_Solver.cpp
    src/02_CARE.cpp)

add_executable(03_inverted_pendulum
    src/class/model/InvertedPendulum.cpp
    src/class/algorithm/CARE_Solver.cpp
    src/class/controller/StateFeedbackLQR.cpp
    src/03_inverted_pendulum.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
