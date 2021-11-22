#ifndef INVERTED_PENDULUM_LQR_H
#define INVERTED_PENDULUM_LQR_H

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <iostream>
#include <mutex>
#include <string>

#include "ss_algorithm/control/StateFeedbackLQR.h"
#include "ss_algorithm/general/RobotSimCppGeneral.h"
#include "ss_model/inverted_pendulum/InvertedPendulum.h"

class InvertedPendulumLQR
{
public:
    InvertedPendulumLQR(const std::string& invertedPendulumName,
                        const std::string& pendulumJointName, double period,
                        const InvertedPendulum& pendulumModel, const std::string& modelStateTopic,
                        const std::string& jointStateTopic, const std::string& controlTopic);
    virtual ~InvertedPendulumLQR();
    virtual void startControl();

private:
    virtual void initialization();
    virtual void callbackModelState(const gazebo_msgs::ModelStatesConstPtr& msg);
    virtual void callbackJointState(const sensor_msgs::JointStateConstPtr& msg);
    virtual void periodicTask(const ros::TimerEvent& timerEvent);

    ros::NodeHandle m_nodeHandler;
    ros::CallbackQueue m_customQueue;
    ros::AsyncSpinner m_asyncSpinner;
    ros::Subscriber m_modelStatesSub;
    ros::Subscriber m_jointStateSub;
    ros::Publisher m_controlPub;
    ros::Timer m_timer;

    bool m_isModelStateValid;
    bool m_isJointStateValid;

    std::string m_invertedPendulumName;
    std::string m_pendulumJointName;
    double m_period;

    std::recursive_mutex m_mutex;

    InvertedPendulum m_pendulumModel;
    StateFeedbackLQR m_LQR;

    std::string m_modelStateTopic;
    std::string m_jointStateTopic;
    std::string m_controlTopic;

    double m_cartPosition = 0.0;
    double m_cartVelocity = 0.0;
    double m_pendulumAngle = 0.0;
    double m_pendulumAngularVelocity = 0.0;
    geometry_msgs::Twist m_controlMsg;
};

#endif