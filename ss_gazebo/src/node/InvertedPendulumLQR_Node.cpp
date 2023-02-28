#include "ss_gazebo/InvertedPendulumLQR.h"
#include "ss_model/inverted_pendulum/InvertedPendulum.h"

int main(int argc, char** argv)
{
    // Init rclcpp
    rclcpp::init(argc, argv);

    // Set Gazebo inverted pendulum LQR controller node
    auto node = std::make_shared<InvertedPendulumLQR>();
    node->startControl();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}