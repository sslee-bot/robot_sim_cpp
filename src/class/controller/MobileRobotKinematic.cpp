#include "controller/MobileRobotKinematic.h"

MobileRobotKinematic::MobileRobotKinematic(double gamma_1, double gamma_2, double h)
    : m_gamma_1(gamma_1), m_gamma_2(gamma_2), m_h(h)
{
    // TODO: Check validities of the controller parameters
}

MobileRobotKinematic::~MobileRobotKinematic()
{
}

Eigen::Vector2d MobileRobotKinematic::generateControlInput(const Eigen::Vector3d& currentState,
                                                           const Eigen::Vector3d& desiredState)
{
    double e, phi, alpha;

    Eigen::Vector3d delta = desiredState - currentState;
    delta[2] = wrapAngle(delta[2]);

    e = delta.head(2).norm();
    phi = wrapAngle(desiredState[2] - std::atan2(delta[1], delta[0]));
    alpha = wrapAngle(phi - delta[2]);

    double linearVel = 0.0, angularVel = 0.0;
    linearVel = m_gamma_1 * e * std::cos(alpha);
    if (alpha == 0.0) {
        angularVel = 0.0;
    }
    else {
        angularVel = -m_gamma_2 * alpha -
                     m_gamma_1 * std::cos(alpha) * std::sin(alpha) / alpha * (alpha + m_h * phi);
    }

    Eigen::Vector2d ret;
    ret << linearVel, angularVel;
    return ret;
}
