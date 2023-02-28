cmake_minimum_required(VERSION 3.5)
project(ss_gazebo)

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
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ss_algorithm REQUIRED)
find_package(ss_model REQUIRED)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## Specify additional locations of header files
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Link Gazebo dev libraries
link_directories(${gazebo_dev_LIBRARY_DIRS})

## Dependencies
set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ss_algorithm
  ss_model
)

## Gazebo plugins and nodes
add_executable(inverted_pendulum_LQR_node src/class/InvertedPendulumLQR.cpp
                                          src/node/InvertedPendulumLQR_Node.cpp)
target_include_directories(inverted_pendulum_LQR_node PUBLIC include)
target_link_libraries(inverted_pendulum_LQR_node ${GAZEBO_LIBRARIES})
ament_target_dependencies(inverted_pendulum_LQR_node ${DEPENDENCIES})
           
add_executable(wheeled_mobile_robot_pose_control_node
                src/class/WheeledMobileRobotPoseControl.cpp
                src/node/WheeledMobileRobotPoseControlNode.cpp)
target_include_directories(wheeled_mobile_robot_pose_control_node PUBLIC include)
target_link_libraries(wheeled_mobile_robot_pose_control_node ${GAZEBO_LIBRARIES})
ament_target_dependencies(wheeled_mobile_robot_pose_control_node ${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
    DIRECTORY include/
    DESTINATION include
)

## Plugins
# install(
#   TARGETS
#     # TODO: Add plugin name
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(TARGETS inverted_pendulum_LQR_node wheeled_mobile_robot_pose_control_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY map worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

ament_package()