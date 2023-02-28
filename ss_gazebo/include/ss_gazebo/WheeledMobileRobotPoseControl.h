#ifndef SS_GAZEBO_WHEELED_MOBILE_ROBOT_POSE_CONTROL_H
#define SS_GAZEBO_WHEELED_MOBILE_ROBOT_POSE_CONTROL_H

// #include <tf2/convert.h>
#include <tf2/utils.h>

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <gazebo/common/common.hh>
// #include <gazebo/gazebo.hh>

#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ss_algorithm/API/RobotSimCppGeneral.h"
#include "ss_algorithm/control/ControlAPI.h"
#include "ss_model/wheeled_mobile_robot/WheeledMobileRobot.h"

using namespace std::chrono_literals;

const double POSITION_ERROR_UPPER = 0.3;
const double ANGLE_ERROR_UPPER = 0.3;

class WheeledMobileRobotPoseControl : public rclcpp::Node
{
public:
    WheeledMobileRobotPoseControl();
    virtual ~WheeledMobileRobotPoseControl();
    virtual void registerController(std::shared_ptr<WheeledMobileRobotController> pController);
    virtual void startControl();

private:
    virtual void callbackModelState(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
    virtual void callbackTargetPose(const geometry_msgs::msg::Twist::SharedPtr msg);
    virtual void periodicTask();

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr m_modelStatesSub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_targetStateSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_controlPub;
    rclcpp::TimerBase::SharedPtr m_timer;

    bool m_isModelStateValid;
    bool m_isTargetStateValid;
    bool m_isArrive;

    std::string m_robotModelName;
    double m_period;

    std::recursive_mutex m_mutex;

    std::shared_ptr<WheeledMobileRobotController> m_pController;

    std::string m_modelStateTopic;
    std::string m_targetStateTopic;
    std::string m_controlTopic;

    geometry_msgs::msg::Twist m_controlMsg;

    Eigen::Vector3d m_currentPose;
    Eigen::Vector3d m_desiredPose;
};

#endif
