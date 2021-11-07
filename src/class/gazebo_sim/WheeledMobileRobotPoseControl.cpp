#include "gazebo_sim/WheeledMobileRobotPoseControl.h"

WheeledMobileRobotPoseControl::WheeledMobileRobotPoseControl(const std::string& robotModelName,
                                                             double period,
                                                             const MobileRobotKinematic& controller,
                                                             const std::string& modelStateTopic,
                                                             const std::string& targetStateTopic,
                                                             const std::string& controlTopic)
    : m_nodeHandler(""),
      m_customQueue(),
      m_asyncSpinner(0, &m_customQueue),
      m_modelStatesSub(),
      m_targetStateSub(),
      m_controlPub(),
      m_timer(),
      m_isModelStateValid(false),
      m_isTargetStateValid(true),
      m_isArrive(true),
      m_robotModelName(robotModelName),
      m_period(period),
      m_controller(controller),
      m_modelStateTopic(modelStateTopic),
      m_targetStateTopic(targetStateTopic),
      m_controlTopic(controlTopic),
      m_controlMsg()
{
    initialization();
}

WheeledMobileRobotPoseControl::~WheeledMobileRobotPoseControl()
{
    m_asyncSpinner.stop();
    m_timer.stop();
    m_modelStatesSub.shutdown();
    m_controlPub.shutdown();
    m_nodeHandler.shutdown();
}

void WheeledMobileRobotPoseControl::startControl()
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Check model state validity
    if (!m_isModelStateValid || !m_isTargetStateValid) {
        ROS_ERROR_STREAM("[robot_sim_cpp] Model or target state is not valid.");
        return;
    }

    ROS_INFO_STREAM("[robot_sim_cpp] Enable kinematic controller for the mobile robot.");

    m_timer.start();
}

void WheeledMobileRobotPoseControl::initialization()
{
    // Custom queue
    m_nodeHandler.setCallbackQueue(&m_customQueue);

    // Spinner
    m_asyncSpinner.start();

    // Subscriber
    m_modelStatesSub = m_nodeHandler.subscribe<gazebo_msgs::ModelStates>(
        m_modelStateTopic, 10, &WheeledMobileRobotPoseControl::callbackModelState, this);
    m_targetStateSub = m_nodeHandler.subscribe<geometry_msgs::Twist>(
        m_targetStateTopic, 10, &WheeledMobileRobotPoseControl::callbackTargetPose, this);

    // Publisher
    m_controlPub = m_nodeHandler.advertise<geometry_msgs::Twist>(m_controlTopic, 10);

    // Timer
    m_timer = m_nodeHandler.createTimer(ros::Duration(m_period),
                                        &WheeledMobileRobotPoseControl::periodicTask, this);
    m_timer.stop();

    // Pose vectors
    m_currentPose.setZero();
    m_desiredPose.setZero();

    ROS_INFO_STREAM("[robot_sim_cpp] Kinematic controller for mobile robot is initialized.");
}

void WheeledMobileRobotPoseControl::callbackModelState(const gazebo_msgs::ModelStatesConstPtr& msg)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Get index
    auto nameVec = msg->name;
    auto iter = std::find(nameVec.begin(), nameVec.end(), m_robotModelName);
    int index;
    if (iter != nameVec.end()) {
        index = iter - nameVec.begin();
    }
    else {
        ROS_ERROR_STREAM_THROTTLE(
            5, "[robot_sim_cpp] No model name matched. m_robotModelName: " << m_robotModelName);
        return;
    }

    // Update model state validity
    auto robotModelState = msg->pose[index];
    if (std::isnan(robotModelState.position.x)) {
        m_isModelStateValid = false;
        ROS_WARN_STREAM_THROTTLE(5, "[robot_sim_cpp] robot model state has invalid (nan) value.");
        return;
    }
    else {
        m_isModelStateValid = true;
    }

    // Get pose of the robot
    m_currentPose[0] = robotModelState.position.x;
    m_currentPose[1] = robotModelState.position.y;
    m_currentPose[2] = wrapAngle(tf::getYaw(robotModelState.orientation));
}

void WheeledMobileRobotPoseControl::callbackTargetPose(const geometry_msgs::TwistConstPtr& msg)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Update target state validity
    if (std::isnan(msg->linear.x)) {
        m_isTargetStateValid = false;
        ROS_WARN_STREAM_THROTTLE(5, "[robot_sim_cpp] target state has invalid (nan) value.");
        return;
    }
    else {
        m_isTargetStateValid = true;
    }

    // Get target pose
    m_desiredPose[0] = msg->linear.x;
    m_desiredPose[1] = msg->linear.y;
    m_desiredPose[2] = wrapAngle(msg->angular.z);

    // Update arrive flag
    m_isArrive = false;
}

void WheeledMobileRobotPoseControl::periodicTask(const ros::TimerEvent& timerEvent)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    if (!m_isArrive) {
        // Check model and target state validities
        if (!m_isModelStateValid || !m_isTargetStateValid) {
            ROS_WARN_STREAM_THROTTLE(5, "[robot_sim_cpp] Model or target state is not valid.");
            return;
        }

        // Check if robot arrived target
        double positionError = (m_currentPose.head(2) - m_desiredPose.head(2)).norm();
        double angleError = wrapAngle(m_currentPose[2] - m_desiredPose[2]);
        if (positionError < 0.1 && std::abs(angleError) < 0.5) {
            ROS_INFO_STREAM("[robot_sim_cpp] Robot has arrived the target."
                            << std::endl
                            << "Current pose: x: " << m_currentPose[0] << " y: " << m_currentPose[1]
                            << " theta: " << m_currentPose[2] << std::endl
                            << "Target pose: x: " << m_desiredPose[0] << " y: " << m_desiredPose[1]
                            << " theta: " << m_desiredPose[2]);
            m_isArrive = true;
        }

        // Calculate control input
        auto control = m_controller.generateControlInput(m_currentPose, m_desiredPose);

        // Publish
        m_controlMsg.linear.x = control[0];
        m_controlMsg.angular.z = wrapAngle(control[1]);
        m_controlPub.publish(m_controlMsg);
    }
}
