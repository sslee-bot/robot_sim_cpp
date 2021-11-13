#include "controller/WheeledMobileRobotController/Kanayama1990.h"

Kanayama1990::Kanayama1990(double k1, double k2, double k3) : m_k1(k1), m_k2(k2), m_k3(k3)
{
}

Kanayama1990::~Kanayama1990()
{
}

Eigen::Vector2d Kanayama1990::poseControl(const Eigen::Vector3d& currentState,
                                          const Eigen::Vector3d& desiredState)
{
    double linearVelocityRef = 0.0, angularVelocityRef = 0.0;
    Eigen::Vector3d stateError = desiredState - currentState;

    double currentAngle = currentState[2];
    double directionAngle = std::atan2(stateError[1], stateError[0]);
    double angleDiff = directionAngle - currentAngle;

    linearVelocityRef = (std::cos(angleDiff) > 0.0) ? 0.5 : -0.5;
    angularVelocityRef = (std::tan(angleDiff) > 0.0) ? 0.5 : -0.5;

    return poseVelocityControl(linearVelocityRef, angularVelocityRef, currentState, desiredState);
}

Eigen::Vector2d Kanayama1990::poseVelocityControl(double linearVelocityRef,
                                                  double angularVelocityRef,
                                                  const Eigen::Vector3d& currentState,
                                                  const Eigen::Vector3d& desiredState)
{
    double currentAngle = currentState[2];

    Eigen::Matrix3d T;
    T << std::cos(currentAngle), std::sin(currentAngle), 0.0, -std::sin(currentAngle),
        std::cos(currentAngle), 0.0, 0.0, 0.0, 1.0;

    Eigen::Vector3d postureError = T * (desiredState - currentState);
    postureError[2] = wrapAngle(postureError[2]);

    double linearVel = 0.0, angularVel = 0.0;
    linearVel = linearVelocityRef * std::cos(postureError[2]) + m_k1 * postureError[0];
    angularVel = angularVelocityRef +
                 linearVelocityRef * (m_k2 * postureError[1] + m_k3 * std::sin(postureError[2]));

    Eigen::Vector2d ret;
    ret << linearVel, angularVel;
    return ret;
}
