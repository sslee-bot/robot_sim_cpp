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
      m_isModelStateValid(false),
      m_isJointStateValid(false),
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

    // Check model and joint state validities
    if (!m_isModelStateValid || !m_isJointStateValid) {
        ROS_ERROR_STREAM("[robot_sim_cpp] Model or joint state was not received yet.");
        return;
    }

    ROS_INFO_STREAM("[robot_sim_cpp] Enable LQR controller for the inverted pendulum.");

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

    ROS_INFO_STREAM("[robot_sim_cpp] LQR controller for the inverted pendulum is initialized.");
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

    // Update model state validity
    double cartPosition = msg->pose[index].position.x;
    if (std::isnan(cartPosition)) {
        m_isModelStateValid = false;
        ROS_WARN_STREAM_THROTTLE(5, "[robot_sim_cpp] Cart position has invalid (nan) value.");
        return;
    }
    else {
        m_isModelStateValid = true;
    }

    // Get position and velocity
    double cartPrevPosition = m_cartPosition;
    m_cartPosition = cartPosition;
    m_cartVelocity = (cartPosition - cartPrevPosition) / m_period;
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

    // Update joint state validity
    double pendulumAngle = msg->position[index];
    double pendulumAngularVelocity = msg->velocity[index];
    if (std::isnan(pendulumAngle) || std::isnan(pendulumAngularVelocity)) {
        m_isJointStateValid = false;
        ROS_WARN_STREAM_THROTTLE(
            5, "[robot_sim_cpp] Pendulum angle or angular velocity has invalid (nan) value.");
        return;
    }
    else {
        m_isJointStateValid = true;
    }

    // Get angle and angular velocity
    m_pendulumAngle = pendulumAngle;
    m_pendulumAngularVelocity = pendulumAngularVelocity;
}

void InvertedPendulumLQR::periodicTask(const ros::TimerEvent& timerEvent)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Check model and joint state validities
    if (m_isModelStateValid == false || m_isJointStateValid == false) {
        ROS_WARN_STREAM_THROTTLE(5, "[robot_sim_cpp] Model or joint state was not received yet.");
        return;
    }

    // Get pendulum state
    Eigen::Vector4d pendulumState;
    pendulumState << m_cartPosition, m_cartVelocity, m_pendulumAngle, m_pendulumAngularVelocity;

    // Calculate control input
    auto control = (m_LQR.generateControlInput(pendulumState))[0];

    // std::cout << "control input: " << control << std::endl;

    // Publish
    m_controlMsg.linear.x = control;
    m_controlPub.publish(m_controlMsg);
}
