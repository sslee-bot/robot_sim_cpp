#ifndef SS_GAZEBO_LINK_EFFORT_PLUGIN_HH
#define SS_GAZEBO_LINK_EFFORT_PLUGIN_HH

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <string>

#include "ss_algorithm/API/RobotSimCppGeneral.h"

namespace gazebo
{
class LinkEffortPlugin : public ModelPlugin
{
public:
    LinkEffortPlugin();
    virtual ~LinkEffortPlugin();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
    virtual void OnUpdate();
    virtual void effortMessageCallback(const geometry_msgs::TwistConstPtr& msg);

    physics::ModelPtr m_pModel;
    physics::LinkPtr m_pLink;
    event::ConnectionPtr m_pUpdateConnection;

    ros::NodeHandle m_nodeHandler;
    ros::AsyncSpinner m_asyncSpinner;
    ros::Subscriber m_effortSub;

    std::string m_linkName;
    std::string m_topicName;
    geometry_msgs::Twist m_effortMessage;
};
}  // namespace gazebo

#endif