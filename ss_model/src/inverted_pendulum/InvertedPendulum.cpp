#include "ss_model/inverted_pendulum/InvertedPendulum.h"

InvertedPendulum::InvertedPendulum(double initPosition, double initAngle, double cartMass,
                                   double pendulumMass, double frictionCoefficient,
                                   double cartPendulumCenterDistance, double massMomentInertia)
    : m_cartPosition(initPosition),
      m_pendulumAngle(initAngle),
      m_cartMass(cartMass),
      m_pendulumMass(pendulumMass),
      m_frictionCoefficient(frictionCoefficient),
      m_cartPendulumCenterDistance(cartPendulumCenterDistance),
      m_massMomentInertia(massMomentInertia)
{
    m_state << m_cartPosition, m_cartVelocity, m_pendulumAngle, m_pendulumAngularVelocity;

    double p = m_massMomentInertia * (m_cartMass + m_pendulumMass) +
               m_cartMass * m_pendulumMass * std::pow(m_cartPendulumCenterDistance, 2);

    // Matrix A
    m_A.setZero();
    m_A(0, 1) = 1;
    m_A(1, 1) =
        -(m_massMomentInertia + m_pendulumMass * std::pow(m_cartPendulumCenterDistance, 2)) *
        m_frictionCoefficient / p;
    m_A(1, 2) =
        (std::pow(m_pendulumMass, 2) * GRAVITY_ACC * std::pow(m_cartPendulumCenterDistance, 2)) / p;
    m_A(2, 3) = 1;
    m_A(3, 1) = -(m_pendulumMass * m_cartPendulumCenterDistance * m_frictionCoefficient) / p;
    m_A(3, 2) = m_pendulumMass * GRAVITY_ACC * m_cartPendulumCenterDistance *
                (m_cartMass + m_pendulumMass) / p;

    // Matrix B
    m_B.setZero();
    m_B(1, 0) =
        (m_massMomentInertia + m_pendulumMass * std::pow(m_cartPendulumCenterDistance, 2)) / p;
    m_B(3, 0) = m_pendulumMass * m_cartPendulumCenterDistance / p;

    // Matrix C
    m_C.setZero();
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

Eigen::Vector4d InvertedPendulum::stateVector()
{
    return m_state;
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

void InvertedPendulum::timeUpdate(double input, double timeStep)
{
    m_forceToCart = input;

    // Update state
    m_state += timeStep * (m_A * m_state + m_B * m_forceToCart);

    m_cartPosition = m_state[0];
    m_cartVelocity = m_state[1];
    m_pendulumAngle = m_state[2];
    m_pendulumAngularVelocity = m_state[3];
}