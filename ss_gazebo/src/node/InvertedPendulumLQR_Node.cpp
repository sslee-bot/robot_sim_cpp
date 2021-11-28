#include "ss_gazebo/InvertedPendulumLQR.h"
#include "ss_model/inverted_pendulum/InvertedPendulum.h"

int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "inverted_pendulum_LQR_node");
    ros::NodeHandle nodeHandler("~");

    // Physical values of pendulum
    double cartMass, pendulumMass, frictionCoefficient, cartPendulumCenterDistance,
        massMomentInertia;

    nodeHandler.param("inverted_pendulum_LQR/cart_mass", cartMass, 0.5);
    nodeHandler.param("inverted_pendulum_LQR/pendulum_mass", pendulumMass, 0.2);
    nodeHandler.param("inverted_pendulum_LQR/friction_coefficient", frictionCoefficient, 0.1);
    nodeHandler.param("inverted_pendulum_LQR/cart_pendulum_center_distance",
                      cartPendulumCenterDistance, 0.3);
    nodeHandler.param("inverted_pendulum_LQR/mass_moment_inertia", massMomentInertia, 0.006);

    ROS_INFO_STREAM("[robot_sim_cpp] Inverted pendulum parameters are set for controller."
                    << std::endl
                    << "cart_mass: " << cartMass << std::endl
                    << "pendulum_mass: " << pendulumMass << std::endl
                    << "friction_coefficient: " << frictionCoefficient << std::endl
                    << "cart_pendulum_center_distance: " << cartPendulumCenterDistance << std::endl
                    << "massMomentInertia: " << massMomentInertia);

    // Set pendulum model
    InvertedPendulum pendulumModel(0.0, 0.0, cartMass, pendulumMass, frictionCoefficient,
                                   cartPendulumCenterDistance, massMomentInertia);

    // Set Gazebo inverted pendulum LQR controller object
    InvertedPendulumLQR IPLQR("inverted_pendulum", "pendulum_joint", 0.001, pendulumModel,
                              "/gazebo/model_states", "/joint_states", "/target_position",
                              "/cart_effort");

    // Start control
    ros::Duration(2.0).sleep();
    IPLQR.startControl();

    ros::waitForShutdown();
    return 1;
}