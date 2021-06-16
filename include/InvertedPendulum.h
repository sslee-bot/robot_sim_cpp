/**
 * @file InvertedPendulum.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Inverted pendulum model based on following material.
 * https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
 * @version 1.0
 * @date 2021-06-16
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef INVERTED_PENDULUM_H
#define INVERTED_PENDULUM_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

const double gravitationalAcc = 9.8;

class InvertedPendulum
{
public:
    InvertedPendulum(double timeStep, double initPosition = 0.0, double initAngle = 0.0);
    virtual ~InvertedPendulum();
    virtual Eigen::Vector2d outputVector();
    virtual Eigen::Matrix<double, 4, 4> getMatrixA();
    virtual Eigen::Matrix<double, 4, 1> getMatrixB();
    virtual Eigen::Matrix<double, 2, 4> getMatrixC();
    virtual void timeUpdate(double input);

private:
    // Intrinsic constants
    double m_cartMass = 0.5, m_pendulumMass = 0.2;  // unit: kg
    double m_frictionCoefficient = 0.1;             // unit: N/m/sec
    double m_cartPendulumCenterDistance = 0.3;      // unit: m
    double m_massMomentInertia = 0.006;             // unit: kg * m^2

    // System input
    double m_forceToCart = 0.0;

    // System states
    double m_cartPosition, m_pendulumAngle;
    double m_cartVelocity = 0.0, m_pendulumAngularVelocity = 0.0;

    // Time step
    double m_timeStep;

    // System matrices
    Eigen::Matrix<double, 4, 4> m_A;
    Eigen::Matrix<double, 4, 1> m_B;
    Eigen::Matrix<double, 2, 4> m_C;
};

#endif