#include "gazebo_sim/WheeledMobileRobotPoseControl.h"

int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "wheeled_mobile_robot_pose_control_node");
    ros::NodeHandle nodeHandler("~");

    // Set controller parameters
    double gamma_1, gamma_2, h;

    nodeHandler.param("wheeled_mobile_robot_pose_control/gamma_1", gamma_1, 1.5);
    nodeHandler.param("wheeled_mobile_robot_pose_control/gamma_2", gamma_2, 1.0);
    nodeHandler.param("wheeled_mobile_robot_pose_control/h", h, 2.0);

    ROS_INFO_STREAM("[robot_sim_cpp] Kinematic controller for the robot are set."
                    << std::endl
                    << "gamma_1: " << gamma_1 << std::endl
                    << "gamma_2: " << gamma_2 << std::endl
                    << "h: " << h);

    // Set controller
    MobileRobotKinematic controller(gamma_1, gamma_2, h);

    // Set Gazebo robot pose controller object
    WheeledMobileRobotPoseControl robotPoseController(
        "jackal", 0.02, controller, "/gazebo/model_states", "/target_pose", "/cmd_vel");

    // Start control
    ros::Duration(2.0).sleep();
    robotPoseController.startControl();

    ros::waitForShutdown();
    return 1;
}
