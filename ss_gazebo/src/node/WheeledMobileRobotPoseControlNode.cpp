#include "ss_gazebo/WheeledMobileRobotPoseControl.h"

int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "wheeled_mobile_robot_pose_control_node");
    ros::NodeHandle nodeHandler("~");

    // Set controller parameters
    double gamma_1, gamma_2, h;
    double k, mu;

    nodeHandler.param("wheeled_mobile_robot_pose_control/Jang2009/gamma_1", gamma_1, 0.3);
    nodeHandler.param("wheeled_mobile_robot_pose_control/Jang2009/gamma_2", gamma_2, 3.0);
    nodeHandler.param("wheeled_mobile_robot_pose_control/Jang2009/h", h, 1.0);
    nodeHandler.param("wheeled_mobile_robot_pose_control/Kim2002_1/k", k, 0.5);
    nodeHandler.param("wheeled_mobile_robot_pose_control/Kim2002_1/mu", mu, 1.0);
    nodeHandler.param("wheeled_mobile_robot_pose_control/Kim2002_2/k", k, 0.5);
    nodeHandler.param("wheeled_mobile_robot_pose_control/Kim2002_2/mu", mu, 1.0);

    // Set controller
    std::shared_ptr<WheeledMobileRobotController> pController;

    ros::Duration(2.0).sleep();

    while (true) {
        // TODO: make it easy to see on terminal
        int controllerCode;
        std::cout << "Select controller" << std::endl << std::endl;

        std::cout << "1. Jang2009" << std::endl;
        std::cout << "2. Kim2002_1" << std::endl;
        std::cout << "3. Kim2002_2" << std::endl;
        std::cout << std::endl;

        std::cout << "Enter number: ";
        std::cin >> controllerCode;

        if (controllerCode == 1) {
            pController = std::make_shared<Jang2009>(gamma_1, gamma_2, h);

            ROS_INFO_STREAM("[robot_sim_cpp] Kinematic controller for the robot are set."
                            << std::endl
                            << "gamma_1: " << gamma_1 << std::endl
                            << "gamma_2: " << gamma_2 << std::endl
                            << "h: " << h);
            break;
        }
        else if (controllerCode == 2) {
            pController = std::make_shared<Kim2002_1>(k, mu);

            ROS_INFO_STREAM("[robot_sim_cpp] Kinematic controller for the robot are set."
                            << std::endl
                            << "k: " << k << std::endl
                            << "mu: " << mu);
            break;
        }
        else if (controllerCode == 3) {
            pController = std::make_shared<Kim2002_2>(k, mu);

            ROS_INFO_STREAM("[robot_sim_cpp] Kinematic controller for the robot are set."
                            << std::endl
                            << "k: " << k << std::endl
                            << "mu: " << mu);
            break;
        }
        else {
            std::cout << "Invalid number. Please try again." << std::endl << std::endl;
        }
    }

    // Set Gazebo robot pose controller object
    WheeledMobileRobotPoseControl robotPoseController(
        "jackal", 0.02, pController, "/gazebo/model_states", "/target_pose", "/cmd_vel");

    // Start control
    ros::Duration(2.0).sleep();
    robotPoseController.startControl();

    ros::waitForShutdown();
    return 1;
}
