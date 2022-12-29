#include "ss_gazebo/InvertedPendulumLQR.h"

using std::placeholders::_1;

InvertedPendulumLQR::InvertedPendulumLQR(const std::string& invertedPendulumName,
                                         const std::string& pendulumJointName, double period,
                                         const std::string& modelStateTopic,
                                         const std::string& jointStateTopic,
                                         const std::string& targetPositionTopic,
                                         const std::string& controlTopic)
    : Node("inverted_pendulum_lqr_node"),
      m_modelStatesSub(),
      m_jointStateSub(),
      m_targetPositionSub(),
      m_controlPub(),
      m_timer(),
      m_isModelStateValid(false),
      m_isJointStateValid(false),
      m_isTargetPositionValid(true),
      m_invertedPendulumName(invertedPendulumName),
      m_pendulumJointName(pendulumJointName),
      m_period(period),
      m_modelStateTopic(modelStateTopic),
      m_jointStateTopic(jointStateTopic),
      m_targetPositionTopic(targetPositionTopic),
      m_controlTopic(controlTopic),
      m_controlMsg()
{
    // Load params
    double cartMass, pendulumMass, frictionCoefficient, cartPendulumCenterDistance,
        massMomentInertia;

    cartMass = this->declare_parameter("cart_mass", 0.5);
    pendulumMass = this->declare_parameter("pendulum_mass", 0.2);
    frictionCoefficient = this->declare_parameter("friction_coefficient", 0.1);
    cartPendulumCenterDistance = this->declare_parameter("cart_pendulum_center_distance", 0.3);
    massMomentInertia = this->declare_parameter("mass_moment_inertia", 0.006);

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "cart_mass: " << cartMass << ", pendulum_mass: " << pendulumMass
                      << ", friction_coefficient: " << frictionCoefficient
                      << ", cart_pendulum_center_distance: " << cartPendulumCenterDistance
                      << ", mass_moment_inertia: " << massMomentInertia);

    // Register pendulum
    InvertedPendulum pendulumModel(0.0, 0.0, cartMass, pendulumMass, frictionCoefficient,
                                   cartPendulumCenterDistance, massMomentInertia);
    this->registerPendulum(pendulumModel);

    // Subscriber
    m_modelStatesSub = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        m_modelStateTopic, 10, std::bind(&InvertedPendulumLQR::callbackModelState, this, _1));
    m_jointStateSub = this->create_subscription<sensor_msgs::msg::JointState>(
        m_jointStateTopic, 10, std::bind(&InvertedPendulumLQR::callbackJointState, this, _1));
    m_targetPositionSub = this->create_subscription<std_msgs::msg::Float32>(
        m_targetPositionTopic, 10,
        std::bind(&InvertedPendulumLQR::callbackTargetPosition, this, _1));

    // Publisher
    m_controlPub = this->create_publisher<geometry_msgs::msg::Wrench>(m_controlTopic, 10);

    // Timer
    m_timer =
        this->create_wall_timer(1s * m_period, std::bind(&InvertedPendulumLQR::periodicTask, this));
    m_timer->cancel();

    // Target position
    m_targetPosition = 0.0;

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "LQR controller for the inverted pendulum is initialized.");
}

InvertedPendulumLQR::~InvertedPendulumLQR()
{
    m_timer->cancel();
}

void InvertedPendulumLQR::registerPendulum(const InvertedPendulum& pendulumModel)
{
    // Register pendulum
    m_pPendulumModel = std::make_shared<InvertedPendulum>(pendulumModel);

    // Initialize feedback controller
    m_pLQR = std::make_shared<StateFeedbackLQR>(m_pPendulumModel->getMatrixA(),
                                                m_pPendulumModel->getMatrixB(),
                                                m_pPendulumModel->getMatrixC());
}

void InvertedPendulumLQR::startControl()
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Check target position validity
    if (!m_isTargetPositionValid) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Target position is not valid.");
        return;
    }

    // Check if pendulum is registered
    if (!m_pPendulumModel || !m_pLQR) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Pendulum has not been registered.");
        return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Enable LQR controller for the inverted pendulum.");

    m_timer->reset();
}

void InvertedPendulumLQR::callbackModelState(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
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
        RCLCPP_ERROR_STREAM_THROTTLE(
            this->get_logger(), *(this->get_clock()), 5000,
            "No model name matched. m_invertedPendulumName: " << m_invertedPendulumName);
        return;
    }

    // Update model state validity
    double cartPosition = msg->pose[index].position.x;
    if (std::isnan(cartPosition)) {
        m_isModelStateValid = false;
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                                    "Cart position has invalid (nan) value.");
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

void InvertedPendulumLQR::callbackJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
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
        RCLCPP_ERROR_STREAM_THROTTLE(
            this->get_logger(), *(this->get_clock()), 5000,
            "No model name matched. m_pendulumJointName: " << m_pendulumJointName);
        return;
    }

    // Update joint state validity
    double pendulumAngle = msg->position[index];
    double pendulumAngularVelocity = msg->velocity[index];
    if (std::isnan(pendulumAngle) || std::isnan(pendulumAngularVelocity)) {
        m_isJointStateValid = false;
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                                    "Pendulum angle or angular velocity has invalid (nan) value.");
        return;
    }
    else {
        m_isJointStateValid = true;
    }

    // Get angle and angular velocity
    m_pendulumAngle = pendulumAngle;
    m_pendulumAngularVelocity = pendulumAngularVelocity;
}

void InvertedPendulumLQR::callbackTargetPosition(const std_msgs::msg::Float32::SharedPtr msg)
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    float targetPosition = msg->data;

    // Update target position validity
    if (std::isnan(targetPosition)) {
        m_isTargetPositionValid = false;
        RCLCPP_WARN_STREAM_THROTTLE(
            this->get_logger(), *(this->get_clock()), 5000,
            "Target position of the inverted pendulum has invalid (nan) value.");
        return;
    }
    else {
        m_isTargetPositionValid = true;
    }

    // Get target position
    m_targetPosition = targetPosition;
}

void InvertedPendulumLQR::periodicTask()
{
    std::unique_lock<std::recursive_mutex> lock(m_mutex);

    // Check model and joint state validities
    if (!m_isModelStateValid || !m_isJointStateValid) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                                    "Model or joint state has not been received yet.");
        return;
    }

    // Get pendulum state
    Eigen::Vector4d pendulumState;
    pendulumState << m_cartPosition - m_targetPosition, m_cartVelocity, m_pendulumAngle,
        m_pendulumAngularVelocity;

    // Calculate control input
    auto control = (m_pLQR->generateControlInput(pendulumState))[0];

    if (std::isnan(control)) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                                    "NaN control input.");
        return;
    }
    // std::cout << "control input: " << control << std::endl;

    // Publish
    m_controlMsg.force.x = control;
    m_controlMsg.force.y = 0.0;
    m_controlMsg.force.z = 0.0;
    m_controlMsg.torque.x = 0.0;
    m_controlMsg.torque.y = 0.0;
    m_controlMsg.torque.z = 0.0;

    m_controlPub->publish(m_controlMsg);
}
