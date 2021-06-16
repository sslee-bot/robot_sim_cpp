#include "InvertedPendulum.h"

InvertedPendulum::InvertedPendulum(double timeStep, double initPosition, double initAngle)
    : m_timeStep(timeStep), m_cartPosition(initAngle), m_pendulumAngle(initAngle)
{
    double p = m_massMomentInertia * (m_cartMass + m_pendulumMass) +
               m_cartMass * m_pendulumMass * std::pow(m_cartPendulumCenterDistance, 2);

    // Matrix A
    m_A(0, 1) = 1;
    m_A(1, 1) =
        -(m_massMomentInertia + m_pendulumMass * std::pow(m_cartPendulumCenterDistance, 2)) *
        m_frictionCoefficient / p;
    m_A(1, 2) = (std::pow(m_pendulumMass, 2) * gravitationalAcc *
                 std::pow(m_cartPendulumCenterDistance, 2)) /
                p;
    m_A(2, 3) = 1;
    m_A(3, 1) = -(m_pendulumMass * m_cartPendulumCenterDistance * m_frictionCoefficient) / p;
    m_A(3, 2) = m_pendulumMass * gravitationalAcc * m_cartPendulumCenterDistance *
                (m_cartMass + m_pendulumMass) / p;

    // Matrix B
    m_B(1, 0) =
        (m_massMomentInertia + m_pendulumMass * std::pow(m_cartPendulumCenterDistance, 2)) / p;
    m_B(3, 0) = m_pendulumMass * m_cartPendulumCenterDistance / p;

    // Matrix C
    m_C(0, 0) = 1;
    m_C(1, 2) = 1;
}

InvertedPendulum::~InvertedPendulum()
{
}

Eigen::Vector2d InvertedPendulum::outputVector()
{
    Eigen::Vector2d output;
    output << m_cartPosition, m_pendulumAngle;
    return output;
}

Eigen::Matrix<double, 4, 4> InvertedPendulum::getMatrixA()
{
    return m_A;
}

Eigen::Matrix<double, 4, 1> InvertedPendulum::getMatrixB()
{
    return m_B;
}

Eigen::Matrix<double, 2, 4> InvertedPendulum::getMatrixC()
{
    return m_C;
}

void InvertedPendulum::timeUpdate(double input)
{
    m_forceToCart = input;

    // Construct state vector
    Eigen::Vector4d state;
    state << m_cartPosition, m_cartVelocity, m_pendulumAngle, m_pendulumAngularVelocity;

    // Update state
    state += m_timeStep * (m_A * state + m_B * m_forceToCart);
    m_cartPosition = state(0, 0);
    m_cartVelocity = state(1, 0);
    m_pendulumAngle = state(2, 0);
    m_pendulumAngularVelocity = state(3, 0);
}