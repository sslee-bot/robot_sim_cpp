cmake_minimum_required(VERSION 3.0.0)
project(ss_model)

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
)

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(inverted_pendulum src/inverted_pendulum/InvertedPendulum.cpp)
target_link_libraries(inverted_pendulum ${catkin_EXPORTED_TARGETS})
add_dependencies(inverted_pendulum ${catkin_LIBRARIES})

add_library(wheeled_mobile_robot src/wheeled_mobile_robot/WheeledMobileRobot.cpp)
target_link_libraries(wheeled_mobile_robot ${catkin_EXPORTED_TARGETS})
add_dependencies(wheeled_mobile_robot ${catkin_LIBRARIES})
