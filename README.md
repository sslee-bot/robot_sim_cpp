# robot_sim_cpp
## 1. Dependencies
* Eigen3
```
sudo apt-get install libeigen3-dev
```
* For Gazebo simulations
```
sudo apt-get install ros-${ROS_DISTRO}-gazebo-* \
                     ros-${ROS_DISTRO}-velodyne-gazebo-plugins* \
                     ros-${ROS_DISTRO}-hector-gazebo-*
                     ros-${ROS_DISTRO}-jackal-simulator \
                     ros-${ROS_DISTRO}-jackal-desktop \
                     ros-${ROS_DISTRO}-jackal-navigation
```

## 2. Executable binaries
Executable binaries will be created after building the project.
```bash
# At workspace directory
catkin_make
```

### 2.1. Matrix basic
Basic matrix operation examples using Eigen3.
```bash
bin/01_matrix_basic
```

### 2.2. Continuous algebraic Riccati equation (CARE)
CARE solver design.
```bash
bin/02_CARE
```
* [CARE solution](https://en.wikipedia.org/wiki/Algebraic_Riccati_equation#Solution)

### 2.3. Inverted pendulum
Simple simulation of inverted pendulum.
```bash
bin/03_inverted_pendulum
```
* [Inverted Pendulum: System Modeling](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)
* [Linear-quadratic regulator](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)

### 2.4. Wheeled mobile robot pose tracking
Simple pose tracking simulation for wheeled mobile robot.
```bash
bin/04_wheeled_mobile_robot
```
* Select controller by entering code number
    1. [Jang2009](https://www.researchgate.net/publication/224560616_Neuro-fuzzy_Network_Control_for_a_Mobile_Robot)
    2. [Kim2002_1](http://dcsl.gatech.edu/papers/tra02.pdf)
    3. Kim2002_2 (The same material as ```Kim2002_1``` was refered)

## 3. Gazebo simulations
### 3.1. Inverted pendulum
```bash
roslaunch robot_sim_cpp 01_Inverted_pendulum.launch
```

### 3.2. Jackal robot pose tracking simulation
```bash
roslaunch robot_sim_cpp 02_jackal_pose_control.launch
```
* Select controller by entering code number
    1. Jang2009
    2. Kim2002_1
    3. Kim2002_2 (The same material as ```Kim2002_1``` was refered)
* Set target pose by publishing reference values
    * Topic: ```/target_pose``` (type: ```geometry_msgs/Twist```)
    * x position: linear/x
    * y position: linear/y
    * heading angle: angular/z
