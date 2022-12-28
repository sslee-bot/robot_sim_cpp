#include "ss_gazebo/LinkEffortPlugin.hh"

// #include <gazebo/common/common.hh>
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <ignition/math/Vector3.hh>

#include <functional>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/gazebo_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <ignition/math.hh>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "geometry_msgs/msg/twist.hpp"

namespace gazebo_plugins
{
class LinkEffortPluginPrivate
{
public:
    void OnUpdate(const gazebo::common::UpdateInfo& _info);
    void effortMessageCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    gazebo::physics::ModelPtr m_pModel;
    gazebo::physics::LinkPtr m_pLink;
    gazebo::event::ConnectionPtr m_pUpdateConnection;
    gazebo_ros::Node::SharedPtr m_pNode;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_effortSub;
    std::string m_linkName;
    std::string m_topicName;
    geometry_msgs::msg::Twist m_effortMsg;
};

LinkEffortPlugin::LinkEffortPlugin() : impl_(std::make_unique<LinkEffortPluginPrivate>())
{
}

LinkEffortPlugin::~LinkEffortPlugin()
{
}

LinkEffortPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // TODO
}

void LinkEffortPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
    // TODO
}

void LinkEffortPluginPrivate::effortMessageCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // TODO
}

}  // namespace gazebo_plugins

// void LinkEffortPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
// {
//     // Get link name
//     if (_sdf->HasElement("linkName")) {
//         m_linkName = _sdf->Get<std::string>("linkName");
//     }
//     else {
//         ROS_ERROR_STREAM("[robot_sim_cpp] Link name is undefined.");
//         return;
//     }

//     // Get topic name
//     if (_sdf->HasElement("topicName")) {
//         m_topicName = _sdf->Get<std::string>("topicName");
//     }
//     else {
//         ROS_ERROR_STREAM("[robot_sim_cpp] Topic name for link effort is undefined.");
//         return;
//     }

//     // Get link pointer
//     m_pModel = _model;
//     m_pLink = m_pModel->GetLink(m_linkName);

//     if (!m_pLink) {
//         ROS_ERROR_STREAM("[robot_sim_cpp] Invalid link name.");
//         return;
//     }

//     // Link effort subscriber
//     m_effortSub = m_nodeHandler.subscribe<geometry_msgs::Twist>(
//         m_topicName, 2, &LinkEffortPlugin::effortMessageCallback, this);

//     // Connect to the update event
//     m_pUpdateConnection =
//         event::Events::ConnectWorldUpdateBegin(std::bind(&LinkEffortPlugin::OnUpdate, this));
// }

// void LinkEffortPlugin::OnUpdate()
// {
//     ignition::math::Vector3d linearEffort, angularEffort;

//     linearEffort[0] = m_effortMessage.linear.x;
//     linearEffort[1] = m_effortMessage.linear.y;
//     linearEffort[2] = m_effortMessage.linear.z;
//     angularEffort[0] = m_effortMessage.angular.x;
//     angularEffort[1] = m_effortMessage.angular.y;
//     angularEffort[2] = m_effortMessage.angular.z;

//     m_pLink->SetForce(linearEffort);
//     m_pLink->SetTorque(angularEffort);
// }

// void LinkEffortPlugin::effortMessageCallback(const geometry_msgs::TwistConstPtr& msg)
// {
//     m_effortMessage = *msg;
// }

GZ_REGISTER_MODEL_PLUGIN(LinkEffortPlugin)