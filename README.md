# robot_sim_cpp
## 1. Dependencies
* Eigen3
```
sudo apt-get install libeigen3-dev
```
* Packages for Gazebo simulation
```
sudo apt-get install ros-${ROS_DISTRO}-gazebo-* \
                     ros-${ROS_DISTRO}-velodyne-gazebo-plugins* \
                     ros-${ROS_DISTRO}-hector-gazebo-*
```

## 2. Executable binaries
Executable binaries will be created after building the project.
```
# At workspace directory
catkin_make
```
### 2.1. Matrix basic
Basic matrix operation examples using Eigen3.
```
./bin/01_matrix_basic
```
### 2.2. Continuous algebraic Riccati equation (CARE)
CARE solver design.
```
./bin/02_CARE
```
* [CARE solution](https://en.wikipedia.org/wiki/Algebraic_Riccati_equation#Solution)
### 2.3. Inverted pendulum
Simple simulation of inverted pendulum.
```
./bin/03_inverted_pendulum
```
* [Inverted Pendulum: System Modeling](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)
* [Linear-quadratic regulator](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)

## 3. Gazebo simulations
### 3.1. Inverted pendulum
```
roslaunch robot_sim_cpp 01_Inverted_pendulum.launch
```
