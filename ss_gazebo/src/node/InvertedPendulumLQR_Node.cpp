#include "ss_gazebo/InvertedPendulumLQR.h"
#include "ss_model/inverted_pendulum/InvertedPendulum.h"

int main(int argc, char** argv)
{
    // Init rclcpp
    rclcpp::init(argc, argv);

    // Set Gazebo inverted pendulum LQR controller node
    auto node = std::make_shared<InvertedPendulumLQR>("inverted_pendulum", "pendulum_joint", 0.001,
                                                      "/gazebo/model_states", "/joint_states",
                                                      "/target_position", "/cart_effort");
    node->startControl();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}