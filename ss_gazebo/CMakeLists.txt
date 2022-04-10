cmake_minimum_required(VERSION 3.0.0)
project(ss_gazebo)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    std_msgs
    sensor_msgs
    geometry_msgs
    controller_manager
    joint_state_controller
    robot_state_publisher
    ss_algorithm
    ss_model
)
find_package(gazebo REQUIRED)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## catkin specific configuration
catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    CATKIN_DEPENDS roscpp gazebo_ros std_msgs sensor_msgs geometry_msgs message_runtime
                    ss_algorithm ss_model
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Gazebo plugins and nodes
add_library(link_effort src/class/LinkEffortPlugin.cc)
target_link_libraries(link_effort ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(link_effort ${catkin_EXPORTED_TARGETS})

add_executable(inverted_pendulum_LQR_node src/class/InvertedPendulumLQR.cpp
                                            src/node/InvertedPendulumLQR_Node.cpp)
target_link_libraries(inverted_pendulum_LQR_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(inverted_pendulum_LQR_node ${catkin_EXPORTED_TARGETS})

add_executable(wheeled_mobile_robot_pose_control_node
                src/class/WheeledMobileRobotPoseControl.cpp
                src/node/WheeledMobileRobotPoseControlNode.cpp)
target_link_libraries(wheeled_mobile_robot_pose_control_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(wheeled_mobile_robot_pose_control_node ${catkin_EXPORTED_TARGETS})                
