#include "ss_algorithm/control/WheeledMobileRobotController/Kim2002_1.h"

Kim2002_1::Kim2002_1(double k, double mu) : m_k(k), m_mu(mu)
{
}

Kim2002_1::~Kim2002_1()
{
}

Eigen::Vector2d Kim2002_1::poseControl(const Eigen::Vector3d& currentState,
                                       const Eigen::Vector3d& desiredState)
{
    Eigen::Matrix2d rotationMatrix;
    rotationMatrix << std::cos(desiredState[2]), std::sin(desiredState[2]),
        -std::sin(desiredState[2]), std::cos(desiredState[2]);
    Eigen::Vector3d stateError = currentState - desiredState;

    Eigen::Vector2d rotatedPosition = rotationMatrix * stateError.head(2);

    double x, y, theta;
    x = rotatedPosition[0];
    y = rotatedPosition[1];
    theta = stateError[2];

    double x1, x2, x3;
    x1 = x * std::cos(theta) + y * std::sin(theta);
    x2 = theta;
    x3 = x * std::sin(theta) - y * std::cos(theta);

    double s = x3 - 0.5 * x1 * x2;

    double linearVel = 0.0, angularVel = 0.0;
    linearVel = -m_k * x1 + m_mu * s * x2 / (std::pow(x1, 2) + std::pow(x2, 2));
    angularVel = -m_k * x2 - m_mu * s * x1 / (std::pow(x1, 2) + std::pow(x2, 2));

    Eigen::Vector2d ret;
    ret << linearVel, angularVel;

    return ret;
}
