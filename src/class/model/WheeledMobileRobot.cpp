#include "model/WheeledMobileRobot.h"

WheeledMobileRobot::WheeledMobileRobot(double initX, double initY, double initTheta,
                                       double centerToWheelAxis)
    : m_xPos(initX), m_yPos(initY), m_theta(initTheta), m_centerToWheelAxis(centerToWheelAxis)
{
    // Construct state vector
    m_state << m_xPos, m_yPos, m_theta;
}

WheeledMobileRobot::~WheeledMobileRobot()
{
}

Eigen::Vector3d WheeledMobileRobot::stateVector()
{
    return m_state;
}

void WheeledMobileRobot::timeUpdate(Eigen::Vector2d input, double timeStep)
{
    Eigen::Matrix<double, 3, 2> S;
    S << std::cos(m_theta), -m_centerToWheelAxis * std::sin(m_theta), std::sin(m_theta),
        m_centerToWheelAxis * std::cos(m_theta), 0.0, 1.0;

    // Update state
    m_state += timeStep * (S * input);

    m_xPos = m_state[0];
    m_yPos = m_state[1];
    m_theta = m_state[2];
}
