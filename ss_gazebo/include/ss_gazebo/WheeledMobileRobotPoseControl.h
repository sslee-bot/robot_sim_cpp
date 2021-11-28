#ifndef SS_GAZEBO_WHEELED_MOBILE_ROBOT_POSE_CONTROL_H
#define SS_GAZEBO_WHEELED_MOBILE_ROBOT_POSE_CONTROL_H

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <iostream>
#include <mutex>
#include <string>

#include "ss_algorithm/API/RobotSimCppGeneral.h"
#include "ss_algorithm/control/WheeledMobileRobotController/Jang2009.h"
#include "ss_algorithm/control/WheeledMobileRobotController/Kim2002_1.h"
#include "ss_algorithm/control/WheeledMobileRobotController/Kim2002_2.h"
#include "ss_model/wheeled_mobile_robot/WheeledMobileRobot.h"

const double POSITION_ERROR_UPPER = 0.3;
const double ANGLE_ERROR_UPPER = 0.3;

class WheeledMobileRobotPoseControl
{
public:
    WheeledMobileRobotPoseControl(const std::string& robotModelName, double period,
                                  const std::shared_ptr<WheeledMobileRobotController>& pController,
                                  const std::string& modelStateTopic,
                                  const std::string& targetStateTopic,
                                  const std::string& controlTopic);
    virtual ~WheeledMobileRobotPoseControl();
    virtual void startControl();

private:
    virtual void callbackModelState(const gazebo_msgs::ModelStatesConstPtr& msg);
    virtual void callbackTargetPose(const geometry_msgs::TwistConstPtr& msg);
    virtual void periodicTask(const ros::TimerEvent& timerEvent);

    ros::NodeHandle m_nodeHandler;
    ros::CallbackQueue m_customQueue;
    ros::AsyncSpinner m_asyncSpinner;
    ros::Subscriber m_modelStatesSub;
    ros::Subscriber m_targetStateSub;
    ros::Publisher m_controlPub;
    ros::Timer m_timer;

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

    geometry_msgs::Twist m_controlMsg;

    Eigen::Vector3d m_currentPose;
    Eigen::Vector3d m_desiredPose;
};

#endif
