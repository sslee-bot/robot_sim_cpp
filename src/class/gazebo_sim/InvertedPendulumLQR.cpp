#include "gazebo_sim/InvertedPendulumLQR.h"

InvertedPendulumLQR::InvertedPendulumLQR(const std::string& invertedPendulumName,
                                         const std::string& pendulumJointName, double period,
                                         const StateFeedbackLQR& LQR,
                                         const std::string& modelStateTopic,
                                         const std::string& jointStateTopic,
                                         const std::string& controlTopic)
    : m_nodeHandler(""),
      m_customQueue(),
      m_asyncSpinner(0, &m_customQueue),
      m_modelStatesSub(),
      m_jointStateSub(),
      m_controlPub(),
      m_timer(),
      m_invertedPendulumName(invertedPendulumName),
      m_pendulumJointName(pendulumJointName),
      m_period(period),
      m_LQR(LQR),
      m_modelStateTopic(modelStateTopic),
      m_jointStateTopic(jointStateTopic),
      m_controlTopic(controlTopic),
      m_controlMsg()
{
    initialization();
}

InvertedPendulumLQR::~InvertedPendulumLQR()
{
    m_asyncSpinner.stop();
    m_timer.stop();
    m_modelStatesSub.shutdown();
    m_jointStateSub.shutdown();
    m_controlPub.shutdown();
    m_nodeHandler.shutdown();
}

void InvertedPendulumLQR::startControl()
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    m_timer.start();
}

void InvertedPendulumLQR::initialization()
{
    // Custom queue
    m_nodeHandler.setCallbackQueue(&m_customQueue);

    // Spinner
    m_asyncSpinner.start();

    // Subscriber
    m_modelStatesSub = m_nodeHandler.subscribe<gazebo_msgs::ModelStates>(
        m_modelStateTopic, 10, &InvertedPendulumLQR::callbackModelState, this);
    m_jointStateSub = m_nodeHandler.subscribe<sensor_msgs::JointState>(
        m_jointStateTopic, 10, &InvertedPendulumLQR::callbackJointState, this);

    // Publisher
    m_controlPub = m_nodeHandler.advertise<geometry_msgs::Twist>(m_controlTopic, 10);

    // Timer
    m_timer = m_nodeHandler.createTimer(ros::Duration(m_period), &InvertedPendulumLQR::periodicTask,
                                        this);
    m_timer.stop();
}

void InvertedPendulumLQR::callbackModelState(const gazebo_msgs::ModelStatesConstPtr& msg)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Get index
    auto nameVec = msg->name;
    auto iter = std::find(nameVec.begin(), nameVec.end(), m_invertedPendulumName);
    int index;
    if (iter != nameVec.end()) {
        index = iter - nameVec.begin();
    }
    else {
        ROS_ERROR_STREAM_THROTTLE(5,
                                  "[robot_sim_cpp] No model name matched. m_invertedPendulumName: "
                                      << m_invertedPendulumName);
        return;
    }

    // Get position and velocity
    double cartPrevPosition = m_cartPosition;
    m_cartPosition = msg->pose[index].position.x;
    m_cartVelocity = (m_cartPosition - cartPrevPosition) / m_period;
}

void InvertedPendulumLQR::callbackJointState(const sensor_msgs::JointStateConstPtr& msg)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Get index
    auto nameVec = msg->name;
    auto iter = std::find(nameVec.begin(), nameVec.end(), m_pendulumJointName);
    int index;
    if (iter != nameVec.end()) {
        index = iter - nameVec.begin();
    }
    else {
        ROS_ERROR_STREAM_THROTTLE(5, "[robot_sim_cpp] No model name matched. m_pendulumJointName: "
                                         << m_pendulumJointName);
        return;
    }

    // Get angle and angular velocity
    m_pendulumAngle = msg->position[index];
    m_pendulumAngularVelocity = msg->velocity[index];
}

void InvertedPendulumLQR::periodicTask(const ros::TimerEvent& timerEvent)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Get pendulum state
    Eigen::Vector4d pendulumState;
    pendulumState << m_cartPosition, m_cartVelocity, m_pendulumAngle, m_pendulumAngularVelocity;

    std::cout << "cartPosition: " << m_cartPosition << std::endl
              << "cartVelocity: " << m_cartVelocity << std::endl
              << "pendulumAngle: " << m_pendulumAngle << std::endl
              << "pendulumAngularVelocity: " << m_pendulumAngularVelocity << std::endl
              << std::endl;

    // Calculate control input
    auto control = (m_LQR.generateControlInput(pendulumState))[0];

    std::cout << "control: " << control << std::endl << std::endl;

    // Publish
    m_controlMsg.linear.x = control;
    m_controlPub.publish(m_controlMsg);
}
