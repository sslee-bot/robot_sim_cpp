/**
 * @file MobileRobotKinematic.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Kinematic controller for simple mobile robot. See IV. A. in the following material.
 * https://www.researchgate.net/publication/224560616_Neuro-fuzzy_Network_Control_for_a_Mobile_Robot
 * @version 0.1
 * @date 2021-11-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef MOBILE_ROBOT_KINEMATIC_CONTROLLER_H
#define MOBILE_ROBOT_KINEMATIC_CONTROLLER_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "RobotSimCppGeneral.h"

class MobileRobotKinematic
{
public:
    MobileRobotKinematic(double gamma_1, double gamma_2, double h);
    virtual ~MobileRobotKinematic();
    virtual Eigen::Vector2d generateControlInput(const Eigen::Vector3d& currentState,
                                                 const Eigen::Vector3d& desiredState);

private:
    // Controller parameters
    double m_gamma_1, m_gamma_2;
    double m_h;
};

#endif
