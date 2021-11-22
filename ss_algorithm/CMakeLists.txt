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
    LIBRARIES ${PROJECT_NAME}
)

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Library
add_library(${PROJECT_NAME} src/solver/CARE_Solver.cpp
                            src/control/StateFeedbackLQR.cpp
                            src/control/WheeledMobileRobotController/Jang2009.cpp
                            src/control/WheeledMobileRobotController/Kim2002_1.cpp
                            src/control/WheeledMobileRobotController/Kim2002_2.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
