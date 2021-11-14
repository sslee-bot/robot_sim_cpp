/**
 * @file Kim2002_1.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Kinematic controller for simple mobile robot.
 * See III. A. in the following material.
 * http://dcsl.gatech.edu/papers/tra02.pdf
 * @version 1.0
 * @date 2021-11-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef KIM_2002_1_H
#define KIM_2002_1_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "RobotSimCppGeneral.h"
#include "controller/WheeledMobileRobotController.h"

class Kim2002_1 : public WheeledMobileRobotController
{
public:
    Kim2002_1(double k, double mu);
    virtual ~Kim2002_1();
    virtual Eigen::Vector2d poseControl(const Eigen::Vector3d& currentState,
                                        const Eigen::Vector3d& desiredState);

private:
    // Controller parameters
    double m_k, m_mu;
};

#endif
