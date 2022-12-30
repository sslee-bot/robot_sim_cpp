#include "ss_gazebo/WheeledMobileRobotPoseControl.h"

int main(int argc, char** argv)
{
    // Set stdout stream unbuffered
    // setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Init rclcpp
    rclcpp::init(argc, argv);

    // Set Gazebo robot pose controller node
    auto node = std::make_shared<WheeledMobileRobotPoseControl>(
        "jackal", 0.02, "/gazebo/model_states", "/target_pose", "/cmd_vel");

    // Controller and its params
    std::shared_ptr<WheeledMobileRobotController> pController;
    double gamma_1 = node->declare_parameter("gamma_1", 0.3);
    double gamma_2 = node->declare_parameter("gamma_2", 3.0);
    double h = node->declare_parameter("h", 1.0);
    double k = node->declare_parameter("k", 0.5);
    double mu = node->declare_parameter("mu", 1.0);

    rclcpp::sleep_for(2s);

    while (rclcpp::ok()) {
        // TODO: make it easy to see on terminal
        int controllerCode;

        std::cout << std::endl;
        std::cout << "=================" << std::endl;
        std::cout << "Select controller" << std::endl << std::endl;

        std::cout << "1. Jang2009" << std::endl;
        std::cout << "2. Kim2002_1" << std::endl;
        std::cout << "3. Kim2002_2" << std::endl;
        std::cout << "=================" << std::endl;
        std::cout << std::endl;

        std::cout << "Enter number: ";
        std::cin >> controllerCode;

        if (std::cin.fail()) {
            std::cout << "Please enter number (int type)." << std::endl;
            std::cin.clear();
            std::cin.ignore(1000, '\n');
        }
        else if (controllerCode == 1) {
            pController = std::make_shared<Jang2009>(gamma_1, gamma_2, h);
            node->registerController(pController);

            RCLCPP_INFO_STREAM(node->get_logger(), "Set kinematic controller for the robot.");
            RCLCPP_INFO_STREAM(node->get_logger(), "Controller: Jang2009");
            RCLCPP_INFO_STREAM(node->get_logger(),
                               "gamma_1: " << gamma_1 << ", gamma_2: " << gamma_2 << ", h: " << h);
            break;
        }
        else if (controllerCode == 2) {
            pController = std::make_shared<Kim2002_1>(k, mu);
            node->registerController(pController);

            RCLCPP_INFO_STREAM(node->get_logger(), "Set kinematic controller for the robot.");
            RCLCPP_INFO_STREAM(node->get_logger(), "Controller: Kim2002_1");
            RCLCPP_INFO_STREAM(node->get_logger(), "k: " << k << ", mu: " << mu);
            break;
        }
        else if (controllerCode == 3) {
            pController = std::make_shared<Kim2002_2>(k, mu);
            node->registerController(pController);

            RCLCPP_INFO_STREAM(node->get_logger(), "Set kinematic controller for the robot.");
            RCLCPP_INFO_STREAM(node->get_logger(), "Controller: Kim2002_2");
            RCLCPP_INFO_STREAM(node->get_logger(), "k: " << k << ", mu: " << mu);
            break;
        }
        else {
            std::cout << "Invalid number. Please try again." << std::endl;
        }
    }

    // Start control
    node->startControl();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
