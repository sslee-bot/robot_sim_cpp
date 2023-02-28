# robot_sim_cpp
Tested environment: Ubuntu 22.04 + ROS2 Humble

## 1. Requirements
* ROS2 (Humble recommended)
* Eigen3
```
sudo apt-get install libeigen3-dev
```
* For Gazebo simulations
```
sudo apt-get install ros-${ROS_DISTRO}-gazebo-* \
                     ros-${ROS_DISTRO}-velodyne-gazebo-plugins* \
                     xterm
```

* Download [rosbot_ros](https://github.com/husarion/rosbot_ros) package (branch: ```humble```) and its dependencies into ```src/```.
    * [rosbot_controllers](https://github.com/husarion/rosbot_controllers) (branch: ```main```)
    * [rosbot_hardware_interfaces](https://github.com/husarion/rosbot_hardware_interfaces) (branch: ```main```)
    * [ros_components_description](https://github.com/husarion/ros_components_description) (branch: ```ros2```)

## 2. Executable binaries
Executable binaries will be created after building the projects.
```bash
# At workspace directory
colcon build
```

### 2.1. Matrix basic
Basic matrix operation examples using Eigen3.
```bash
./robot_sim_cpp/bin/01_matrix_basic
```

### 2.2. Continuous algebraic Riccati equation (CARE)
CARE solver design.
```bash
./robot_sim_cpp/bin/02_CARE
```
* [CARE solution](https://en.wikipedia.org/wiki/Algebraic_Riccati_equation#Solution)

### 2.3. Inverted pendulum
Simple simulation of inverted pendulum.
```bash
./robot_sim_cpp/bin/03_inverted_pendulum
```
* [Inverted Pendulum: System Modeling](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)
* [Linear-quadratic regulator](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)

### 2.4. Wheeled mobile robot pose tracking
Simple pose tracking simulation for wheeled mobile robot.
```bash
./robot_sim_cpp/bin/04_wheeled_mobile_robot
```
* Select controller by entering code number
    1. [Jang2009](https://www.researchgate.net/publication/224560616_Neuro-fuzzy_Network_Control_for_a_Mobile_Robot)
    2. [Kim2002_1](http://dcsl.gatech.edu/papers/tra02.pdf)
    3. Kim2002_2 (The same material as ```Kim2002_1``` was refered)

### 2.5. 1D object tracking
Simple 1D object tracking simulation using Kalman filter
```bash
./robot_sim_cpp/bin/05_1D_object_tracking
```

## 3. Gazebo simulations
### 3.1. Inverted pendulum
```bash
ros2 launch robot_sim_cpp 01_inverted_pendulum.launch.py 
```
* Set target position by publishing reference value
    * Topic: ```/target_position``` (type: ```std_msgs/msg/Float32```)
    * x position: data

### 3.2. Jackal robot pose tracking simulation
```bash
ros2 launch robot_sim_cpp 02_rosbot_pose_control.launch.py
```
* Select controller by entering code number
    1. Jang2009
    2. Kim2002_1
    3. Kim2002_2
* Set target pose by publishing reference values
    * Topic: ```/target_pose``` (type: ```geometry_msgs/msg/Twist```)
    * x position: linear/x
    * y position: linear/y
    * heading angle: angular/z
