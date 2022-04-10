## See ros package practices at https://github.com/leggedrobotics/ros_best_practices

cmake_minimum_required(VERSION 3.0.0)
project(ss_simulation)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    ss_algorithm
    ss_description
    ss_model
)
find_package(gazebo REQUIRED)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

## OpenCV
# set(OpenCV_DIR [OpenCV.cmake PATH])
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

## catkin specific configuration
catkin_package(
    INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
    CATKIN_DEPENDS roscpp gazebo_ros ss_algorithm ss_description ss_model
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    cv-plot/CvPlot/inc
)

## Declare cpp executables
add_executable(01_matrix_basic src/01_matrix_basic.cpp)
target_link_libraries(01_matrix_basic ${catkin_LIBRARIES})
add_dependencies(01_matrix_basic ${catkin_EXPORTED_TARGETS})

add_executable(02_CARE src/02_CARE.cpp)
target_link_libraries(02_CARE ${catkin_LIBRARIES})
add_dependencies(02_CARE ${catkin_EXPORTED_TARGETS})

add_executable(03_inverted_pendulum src/03_inverted_pendulum.cpp)
target_link_libraries(03_inverted_pendulum ${catkin_LIBRARIES})
add_dependencies(03_inverted_pendulum ${catkin_EXPORTED_TARGETS})

add_executable(04_wheeled_mobile_robot src/04_wheeled_mobile_robot.cpp)
target_link_libraries(04_wheeled_mobile_robot ${catkin_LIBRARIES})
add_dependencies(04_wheeled_mobile_robot ${catkin_EXPORTED_TARGETS})

add_executable(05_1D_object_tracking src/05_1D_object_tracking.cpp)
target_link_libraries(05_1D_object_tracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(05_1D_object_tracking ${catkin_EXPORTED_TARGETS})

set_target_properties(
    01_matrix_basic
    02_CARE
    03_inverted_pendulum
    04_wheeled_mobile_robot
    05_1D_object_tracking
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)
