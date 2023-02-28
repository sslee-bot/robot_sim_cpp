#ifndef SS_GAZEBO_INVERTED_PENDULUM_LQR_H
#define SS_GAZEBO_INVERTED_PENDULUM_LQR_H

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <functional>
// #include <gazebo/common/common.hh>
// #include <gazebo/gazebo.hh>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ss_algorithm/API/RobotSimCppGeneral.h"
#include "ss_algorithm/control/StateFeedbackLQR.h"
#include "ss_model/inverted_pendulum/InvertedPendulum.h"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class InvertedPendulumLQR : public rclcpp::Node
{
public:
    InvertedPendulumLQR();
    virtual ~InvertedPendulumLQR();
    virtual void registerPendulum(const InvertedPendulum& pendulumModel);
    virtual void startControl();

private:
    virtual void callbackModelState(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
    virtual void callbackJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
    virtual void callbackTargetPosition(const std_msgs::msg::Float32::SharedPtr msg);
    virtual void periodicTask();

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr m_modelStatesSub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jointStateSub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_targetPositionSub;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr m_controlPub;
    rclcpp::TimerBase::SharedPtr m_timer;

    bool m_isModelStateValid;
    bool m_isJointStateValid;
    bool m_isTargetPositionValid;

    std::string m_invertedPendulumName;
    std::string m_pendulumJointName;
    double m_period;

    std::recursive_mutex m_mutex;

    std::shared_ptr<InvertedPendulum> m_pPendulumModel;
    std::shared_ptr<StateFeedbackLQR> m_pLQR;

    std::string m_modelStateTopic;
    std::string m_jointStateTopic;
    std::string m_targetPositionTopic;
    std::string m_controlTopic;

    float m_targetPosition = 0.0;

    double m_cartPosition = 0.0;
    double m_cartVelocity = 0.0;
    double m_pendulumAngle = 0.0;
    double m_pendulumAngularVelocity = 0.0;
    geometry_msgs::msg::Wrench m_controlMsg;
};

#endif