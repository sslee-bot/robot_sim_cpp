/**
 * @file Kim2002_1.h
 * @author Sangsu Lee (physism@gmail.com)
 * @brief Kinematic controller for simple mobile robot.
 * See the equation (5) in the following material.
 * http://dcsl.gatech.edu/papers/tra02.pdf
 * @version 1.0
 * @date 2021-11-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef SS_ALGORITHM_KIM_2002_1_H
#define SS_ALGORITHM_KIM_2002_1_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "ss_algorithm/API/RobotSimCppGeneral.h"
#include "ss_algorithm/API/WheeledMobileRobotController.h"

class Kim2002_1 : public WheeledMobileRobotController
{
public:
    Kim2002_1(double k, double mu);
    virtual ~Kim2002_1();
    // virtual void setParams(double k, double mu);
    virtual void setParams(const std::vector<double>& params) override;
    virtual Eigen::Vector2d poseControl(const Eigen::Vector3d& currentState,
                                        const Eigen::Vector3d& desiredState) override;

private:
    // Controller parameters
    double m_k, m_mu;
};

#endif
