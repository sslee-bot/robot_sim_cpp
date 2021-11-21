#include "ss_algorithm/control/WheeledMobileRobotController/Kim2002_2.h"

Kim2002_2::Kim2002_2(double k, double mu) : m_k(k), m_mu(mu)
{
}

Kim2002_2::~Kim2002_2()
{
}

Eigen::Vector2d Kim2002_2::poseControl(const Eigen::Vector3d& currentState,
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

    m_x1 = x * std::cos(theta) + y * std::sin(theta);
    m_x2 = theta;
    m_x3 = x * std::sin(theta) - y * std::cos(theta);

    m_s = m_x3 - 0.5 * m_x1 * m_x2;
    m_v = std::sqrt(std::pow(m_x1, 2) + std::pow(m_x2, 2));

    double linearVel = 0.0, angularVel = 0.0;
    linearVel = -m_k * m_x1 / std::sqrt(std::pow(m_v, 2) + 1.0) + m_mu * saturation(2);
    angularVel = -m_k * m_x2 / std::sqrt(std::pow(m_v, 2) + 1.0) - m_mu * saturation(1);

    Eigen::Vector2d ret;
    ret << linearVel, angularVel;

    return ret;
}

double Kim2002_2::saturation(int i)
{
    double sat, min, sgn;
    if (m_v == 0.0) {
        if (m_s > 0.0) {
            return 1.0;
        }
        else if (m_s < 0.0) {
            return -1.0;
        }
        else {
            return 0.0;
        }
    }
    else {
        min = std::min(1.0, std::abs(m_s / m_v));
        if (m_s / m_v > 0.0) {
            sgn = 1.0;
        }
        else if (m_s / m_v < 0.0) {
            sgn = -1.0;
        }
        else {
            sgn = 0.0;
        }
        sat = min * sgn;
        if (i == 1) {
            return sat * m_x1 / m_v;
        }
        else {
            return sat * m_x2 / m_v;
        }
    }
}
