cmake_minimum_required(VERSION 3.5)
project(ss_algorithm)

## Use C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

## Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## Specify additional locations of header files
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Dependencies
# set(dependencies
#     tf2
#     tf2_ros
# )

## Library
add_library(${PROJECT_NAME} SHARED
    src/solver/CARE_Solver.cpp
    src/control/StateFeedbackLQR.cpp
    src/control/WheeledMobileRobotController/Jang2009.cpp
    src/control/WheeledMobileRobotController/Kim2002_1.cpp
    src/control/WheeledMobileRobotController/Kim2002_2.cpp
    src/estimator/DiscreteLinearKF.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
# ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies()

ament_package()
