/**
 * @file Kim2002_2.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Kinematic controller for simple mobile robot.
 * See the equation (8) in the following material.
 * http://dcsl.gatech.edu/papers/tra02.pdf
 * @version 1.0
 * @date 2021-11-15
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef KIM_2002_2_H
#define KIM_2002_2_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "RobotSimCppGeneral.h"
#include "controller/WheeledMobileRobotController.h"

class Kim2002_2 : public WheeledMobileRobotController
{
public:
    Kim2002_2(double k, double mu);
    virtual ~Kim2002_2();
    virtual Eigen::Vector2d poseControl(const Eigen::Vector3d& currentState,
                                        const Eigen::Vector3d& desiredState);

private:
    virtual double saturation(int i);

    // Controller parameters
    double m_k, m_mu;

    // Internal variables
    double m_x1, m_x2, m_x3;
    double m_s, m_v;
};

#endif
