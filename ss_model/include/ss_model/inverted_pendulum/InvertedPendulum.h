/**
 * @file InvertedPendulum.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Inverted pendulum model based on following material.
 * https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
 * @version 1.0
 * @date 2021-06-28
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef INVERTED_PENDULUM_H
#define INVERTED_PENDULUM_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "ss_algorithm/general/RobotSimCppGeneral.h"

class InvertedPendulum
{
public:
    InvertedPendulum(double initPosition = 0.0, double initAngle = 0.0, double cartMass = 0.5,
                     double pendulumMass = 0.2, double frictionCoefficient = 0.1,
                     double cartPendulumCenterDistance = 0.3, double massMomentInertia = 0.006);
    virtual ~InvertedPendulum();
    virtual Eigen::Vector2d outputVector();
    virtual Eigen::Vector4d stateVector();
    virtual Eigen::Matrix<double, 4, 4> getMatrixA();
    virtual Eigen::Matrix<double, 4, 1> getMatrixB();
    virtual Eigen::Matrix<double, 2, 4> getMatrixC();
    virtual void timeUpdate(double input, double timeStep);

private:
    // System states
    Eigen::Vector4d m_state;
    double m_cartPosition, m_pendulumAngle;
    double m_cartVelocity = 0.0, m_pendulumAngularVelocity = 0.0;

    // Intrinsic constants
    double m_cartMass, m_pendulumMass;    // unit: kg
    double m_frictionCoefficient;         // unit: N/m/sec
    double m_cartPendulumCenterDistance;  // unit: m
    double m_massMomentInertia;           // unit: kg * m^2

    // System input
    double m_forceToCart = 0.0;

    // System matrices
    Eigen::Matrix<double, 4, 4> m_A;
    Eigen::Matrix<double, 4, 1> m_B;
    Eigen::Matrix<double, 2, 4> m_C;
};

#endif