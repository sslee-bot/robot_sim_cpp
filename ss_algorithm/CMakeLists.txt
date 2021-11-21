cmake_minimum_required(VERSION 3.0.0)
project(ss_algorithm)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## catkin specific configuration
catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES care_solver state_feedback_lqr wheeled_mobile_robot_controller
)

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Libraries
add_library(care_solver src/solver/CARE_Solver.cpp)
target_link_libraries(care_solver ${catkin_LIBRARIES})
add_dependencies(care_solver ${catkin_EXPORTED_TARGETS})

add_library(state_feedback_lqr src/control/StateFeedbackLQR.cpp)
target_link_libraries(state_feedback_lqr ${catkin_LIBRARIES})
add_dependencies(state_feedback_lqr ${catkin_EXPORTED_TARGETS})

add_library(wheeled_mobile_robot_controller src/control/WheeledMobileRobotController/Jang2009.cpp
                                            src/control/WheeledMobileRobotController/Kim2002_1.cpp
                                            src/control/WheeledMobileRobotController/Kim2002_2.cpp)
target_link_libraries(wheeled_mobile_robot_controller ${catkin_LIBRARIES})
add_dependencies(wheeled_mobile_robot_controller ${catkin_EXPORTED_TARGETS})

## Install
install(TARGETS care_solver state_feedback_lqr wheeled_mobile_robot_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
