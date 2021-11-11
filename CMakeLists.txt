## See ros package practices at https://github.com/leggedrobotics/ros_best_practices

cmake_minimum_required(VERSION 3.0.0)
project(robot_sim_cpp VERSION 1.0.0)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
)
find_package(gazebo REQUIRED)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## catkin specific configuration
catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    CATKIN_DEPENDS roscpp gazebo_ros std_msgs sensor_msgs geometry_msgs message_runtime
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

## Declare cpp executables
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

add_executable(04_wheeled_mobile_robot
    src/class/model/WheeledMobileRobot.cpp
    src/class/controller/WheeledMobileRobotController/Jang2009.cpp
    src/class/controller/WheeledMobileRobotController/Kanayama1990.cpp
    src/04_wheeled_mobile_robot.cpp)

set_target_properties(
    01_matrix_basic
    02_CARE
    03_inverted_pendulum
    04_wheeled_mobile_robot
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)

## Gazebo plugins and nodes
add_library(link_effort src/class/gazebo_sim/LinkEffortPlugin.cc)
target_link_libraries(link_effort ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(link_effort ${catkin_EXPORTED_TARGETS})

add_executable(inverted_pendulum_LQR_node src/class/model/InvertedPendulum.cpp
                                        src/class/algorithm/CARE_Solver.cpp
                                        src/class/controller/StateFeedbackLQR.cpp
                                        src/class/gazebo_sim/InvertedPendulumLQR.cpp
                                        src/node/InvertedPendulumLQR_Node.cpp)
target_link_libraries(inverted_pendulum_LQR_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(inverted_pendulum_LQR_node ${catkin_EXPORTED_TARGETS})

add_executable(wheeled_mobile_robot_pose_control_node
                src/class/controller/WheeledMobileRobotController/Jang2009.cpp
                src/class/gazebo_sim/WheeledMobileRobotPoseControl.cpp
                src/node/WheeledMobileRobotPoseControlNode.cpp)
target_link_libraries(wheeled_mobile_robot_pose_control_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(wheeled_mobile_robot_pose_control_node ${catkin_EXPORTED_TARGETS})                
