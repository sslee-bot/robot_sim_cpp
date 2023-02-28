## See ros package practices at https://github.com/leggedrobotics/ros_best_practices

cmake_minimum_required(VERSION 3.5)
project(robot_sim_cpp)

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

## Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ss_algorithm REQUIRED)
find_package(ss_description REQUIRED)
# find_package(ss_gazebo REQUIRED)
find_package(ss_model REQUIRED)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${gazebo_dev_LIBRARY_DIRS})

## Specify additional locations of header files
include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

set(DEPENDENCIES
    rclcpp
    ss_algorithm
    # ss_gazebo
    ss_model
)

## Declare cpp executables
add_executable(01_matrix_basic src/01_matrix_basic.cpp)
ament_target_dependencies(01_matrix_basic ${DEPENDENCIES})

add_executable(02_CARE src/02_CARE.cpp)
ament_target_dependencies(02_CARE ${DEPENDENCIES})

add_executable(03_inverted_pendulum src/03_inverted_pendulum.cpp)
ament_target_dependencies(03_inverted_pendulum ${DEPENDENCIES})

add_executable(04_wheeled_mobile_robot src/04_wheeled_mobile_robot.cpp)
ament_target_dependencies(04_wheeled_mobile_robot ${DEPENDENCIES})

add_executable(05_1D_object_tracking src/05_1D_object_tracking.cpp)
ament_target_dependencies(05_1D_object_tracking ${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch maps worlds
    DESTINATION share/${PROJECT_NAME}
)

set(executables
    01_matrix_basic
    02_CARE
    03_inverted_pendulum
    04_wheeled_mobile_robot
    05_1D_object_tracking
)

install(TARGETS ${executables}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

set_target_properties(
    01_matrix_basic
    02_CARE
    03_inverted_pendulum
    04_wheeled_mobile_robot
    05_1D_object_tracking
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

ament_package()
