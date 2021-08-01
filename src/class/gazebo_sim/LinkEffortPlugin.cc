#include "gazebo_sim/LinkEffortPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(LinkEffortPlugin)

LinkEffortPlugin::LinkEffortPlugin()
    : ModelPlugin(), m_nodeHandler(""), m_asyncSpinner(0), m_effortSub()
{
    // Spinner
    m_asyncSpinner.start();
}

LinkEffortPlugin::~LinkEffortPlugin()
{
    m_nodeHandler.shutdown();
}

void LinkEffortPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Get link name
    if (_sdf->HasElement("linkName")) {
        m_linkName = _sdf->Get<std::string>("linkName");
    }
    else {
        ROS_ERROR_STREAM("[robot_sim_cpp] Link name is undefined.");
        return;
    }

    // Get topic name
    if (_sdf->HasElement("topicName")) {
        m_topicName = _sdf->Get<std::string>("topicName");
    }
    else {
        ROS_ERROR_STREAM("[robot_sim_cpp] Topic name for link effort is undefined.");
        return;
    }

    // Get link pointer
    m_pModel = _model;
    m_pLink = m_pModel->GetLink(m_linkName);

    if (!m_pLink) {
        ROS_ERROR_STREAM("[robot_sim_cpp] Invalid link name.");
        return;
    }

    // Link effort subscriber
    m_effortSub = m_nodeHandler.subscribe<geometry_msgs::Twist>(
        m_topicName, 2, &LinkEffortPlugin::effortMessageCallback, this);

    // Connect to the update event
    m_pUpdateConnection =
        event::Events::ConnectWorldUpdateBegin(std::bind(&LinkEffortPlugin::OnUpdate, this));
}

void LinkEffortPlugin::OnUpdate()
{
    ignition::math::Vector3d linearEffort, angularEffort;

    linearEffort[0] = m_effortMessage.linear.x;
    linearEffort[1] = m_effortMessage.linear.y;
    linearEffort[2] = m_effortMessage.linear.z;
    angularEffort[0] = m_effortMessage.angular.x;
    angularEffort[1] = m_effortMessage.angular.y;
    angularEffort[2] = m_effortMessage.angular.z;

    m_pLink->SetForce(linearEffort);
    m_pLink->SetTorque(angularEffort);
}

void LinkEffortPlugin::effortMessageCallback(const geometry_msgs::TwistConstPtr& msg)
{
    m_effortMessage = *msg;
}