/**
 * @file Kanayama1990.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Kinematic controller for simple mobile robot.
 * See equation (8) in the following material.
 * https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=126006
 * @version 0.1
 * @date 2021-11-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef FIERRO_1998_H
#define FIERRO_1998_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "RobotSimCppGeneral.h"
#include "controller/WheeledMobileRobotController.h"

class Kanayama1990 : public WheeledMobileRobotController
{
public:
    Kanayama1990(double k1, double k2, double k3);
    virtual ~Kanayama1990();
    virtual Eigen::Vector2d poseControl(const Eigen::Vector3d& currentState,
                                        const Eigen::Vector3d& desiredState);

private:
    virtual Eigen::Vector2d poseVelocityControl(double linearVelocityRef, double angularVelocityRef,
                                                const Eigen::Vector3d& currentState,
                                                const Eigen::Vector3d& desiredState);

    // Controller parameters
    double m_k1, m_k2, m_k3;
};

#endif
