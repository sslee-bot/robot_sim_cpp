cmake_minimum_required(VERSION 3.0.0)
project(ss_model)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    ss_algorithm
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## catkin specific configuration
catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ss_algorithm
)

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/inverted_pendulum/InvertedPendulum.cpp
                            src/wheeled_mobile_robot/WheeledMobileRobot.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
