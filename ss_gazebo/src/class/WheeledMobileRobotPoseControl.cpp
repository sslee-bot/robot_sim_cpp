#include "ss_gazebo/WheeledMobileRobotPoseControl.h"

WheeledMobileRobotPoseControl::WheeledMobileRobotPoseControl(const std::string& robotModelName,
                                                             double period,
                                                             const std::string& modelStateTopic,
                                                             const std::string& targetStateTopic,
                                                             const std::string& controlTopic)
    : Node("wheel_mobile_robot_pose_control_node"),
      m_modelStatesSub(),
      m_targetStateSub(),
      m_controlPub(),
      m_timer(),
      m_isModelStateValid(false),
      m_isTargetStateValid(true),
      m_isArrive(true),
      m_robotModelName(robotModelName),
      m_period(period),
      m_mutex(),
      m_pController(nullptr),
      m_modelStateTopic(modelStateTopic),
      m_targetStateTopic(targetStateTopic),
      m_controlTopic(controlTopic),
      m_controlMsg()
{
    // Subscriber
    m_modelStatesSub = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        m_modelStateTopic, 10,
        std::bind(&WheeledMobileRobotPoseControl::callbackModelState, this, std::placeholders::_1));
    m_targetStateSub = this->create_subscription<geometry_msgs::msg::Twist>(
        m_targetStateTopic, 10,
        std::bind(&WheeledMobileRobotPoseControl::callbackTargetPose, this, std::placeholders::_1));

    // Publisher
    m_controlPub = this->create_publisher<geometry_msgs::msg::Twist>(m_controlTopic, 10);

    // Timer
    m_timer = this->create_wall_timer(
        1s * m_period, std::bind(&WheeledMobileRobotPoseControl::periodicTask, this));
    m_timer->cancel();

    // Pose vectors
    m_currentPose.setZero();
    m_desiredPose.setZero();

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Kinematic controller node for mobile robot is initialized.");
}

WheeledMobileRobotPoseControl::~WheeledMobileRobotPoseControl()
{
    m_timer->cancel();
}

void WheeledMobileRobotPoseControl::registerController(
    std::shared_ptr<WheeledMobileRobotController> pController)
{
    // m_pController = std::make_shared<WheeledMobileRobotController>(controller);
    m_pController = pController;
}

void WheeledMobileRobotPoseControl::startControl()
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Check controller validity
    if (m_pController == nullptr) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Controller for mobile robot is not set.");
        return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Enable kinematic controller for the mobile robot.");

    m_timer->reset();
}

void WheeledMobileRobotPoseControl::callbackModelState(
    const gazebo_msgs::msg::ModelStates::SharedPtr msg)
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
        RCLCPP_ERROR_STREAM_THROTTLE(
            this->get_logger(), *(this->get_clock()), 5000,
            "No model name matched. m_robotModelName: " << m_robotModelName);
        return;
    }

    // Update model state validity
    auto robotModelState = msg->pose[index];
    if (std::isnan(robotModelState.position.x)) {
        m_isModelStateValid = false;
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                                    "Robot model state has invalid (nan) value.");
        return;
    }
    else {
        m_isModelStateValid = true;
    }

    // Get pose of the robot
    m_currentPose[0] = robotModelState.position.x;
    m_currentPose[1] = robotModelState.position.y;
    m_currentPose[2] = wrapAngle(tf2::getYaw(robotModelState.orientation));

    // tf2::Quaternion quat_tf;
    // tf2::fromMsg(robotModelState.orientation, quat_tf);
    // double roll, pitch, yaw;
    // tf2::Matrix3x3 m(quat_tf);
    // m.getRPY(roll, pitch, yaw);
    // m_currentPose[2] = wrapAngle(yaw);
}

void WheeledMobileRobotPoseControl::callbackTargetPose(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Update target state validity
    if (std::isnan(msg->linear.x)) {
        m_isTargetStateValid = false;
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                                    "Target state has invalid (nan) value.");
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

void WheeledMobileRobotPoseControl::periodicTask()
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    if (!m_isArrive) {
        // Check model and target state validities
        if (!m_isModelStateValid || !m_isTargetStateValid) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                                        "Model or target state is not valid.");
            return;
        }

        // Check if robot arrived target
        double positionError = (m_currentPose.head(2) - m_desiredPose.head(2)).norm();
        double angleError = wrapAngle(m_currentPose[2] - m_desiredPose[2]);
        if (positionError < POSITION_ERROR_UPPER && std::abs(angleError) < ANGLE_ERROR_UPPER) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Robot arrived the target point.");
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Current pose: x: " << m_currentPose[0] << " y: " << m_currentPose[1]
                                                   << " theta: " << m_currentPose[2]);
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Target pose: x: " << m_desiredPose[0] << " y: " << m_desiredPose[1]
                                                  << " theta: " << m_desiredPose[2]);

            // Update arrive flag
            m_isArrive = true;
        }

        // Calculate control input
        auto control = m_pController->poseControl(m_currentPose, m_desiredPose);

        // Set velocity
        m_controlMsg.linear.x = control[0];
        m_controlMsg.angular.z = wrapAngle(control[1]);
    }
    else {
        // Set zero velocity
        m_controlMsg.linear.x = 0.0;
        m_controlMsg.angular.z = 0.0;
    }

    // Publish
    m_controlPub->publish(m_controlMsg);
}
