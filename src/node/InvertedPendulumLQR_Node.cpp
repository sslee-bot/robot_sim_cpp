#include "gazebo_sim/InvertedPendulumLQR.h"

const double gravitationalAcc = 9.8;

int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "inverted_pendulum_LQR_node");

    // Physical values of pendulum
    double cartMass = 0.5;
    double pendulumMass = 0.2;
    double frictionCoefficient = 0.1;
    double cartPendulumCenterDistance = 0.3;
    double massMomentInertia = 0.006;

    // Set LQR controller
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 1> B;
    Eigen::Matrix<double, 2, 4> C;

    double p = massMomentInertia * (cartMass + pendulumMass) +
               cartMass * pendulumMass * std::pow(cartPendulumCenterDistance, 2);

    // Matrix A
    A.setZero();
    A(0, 1) = 1;
    A(1, 1) = -(massMomentInertia + pendulumMass * std::pow(cartPendulumCenterDistance, 2)) *
              frictionCoefficient / p;
    A(1, 2) =
        (std::pow(pendulumMass, 2) * gravitationalAcc * std::pow(cartPendulumCenterDistance, 2)) /
        p;
    A(2, 3) = 1;
    A(3, 1) = -(pendulumMass * cartPendulumCenterDistance * frictionCoefficient) / p;
    A(3, 2) = pendulumMass * gravitationalAcc * cartPendulumCenterDistance *
              (cartMass + pendulumMass) / p;

    // Matrix B
    B.setZero();
    B(1, 0) = (massMomentInertia + pendulumMass * std::pow(cartPendulumCenterDistance, 2)) / p;
    B(3, 0) = pendulumMass * cartPendulumCenterDistance / p;

    // Matrix C
    C.setZero();
    C(0, 0) = 1;
    C(1, 2) = 1;

    StateFeedbackLQR LQR(A, B, C);

    // Set Gazebo inverted pendulum LQR control object
    InvertedPendulumLQR IPLQR("inverted_pendulum", "pendulum_joint", 0.001, LQR,
                              "/gazebo/model_states", "/joint_states", "/cart_effort");

    // Start control
    ros::Duration(2.0).sleep();
    IPLQR.startControl();

    ros::waitForShutdown();
    return 1;
}