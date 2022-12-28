/**
 * @file Jang2009.h
 * @author Sangsu Lee (physism@gmail.com)
 * @brief Kinematic controller for simple mobile robot.
 * See IV. A. in the following material.
 * https://www.researchgate.net/publication/224560616_Neuro-fuzzy_Network_Control_for_a_Mobile_Robot
 * @version 0.1
 * @date 2021-11-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef SS_ALGORITHM_JANG_2009_H
#define SS_ALGORITHM_JANG_2009_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "ss_algorithm/API/RobotSimCppGeneral.h"
#include "ss_algorithm/API/WheeledMobileRobotController.h"

class Jang2009 : public WheeledMobileRobotController
{
public:
    Jang2009(double gamma_1, double gamma_2, double h);
    virtual ~Jang2009();
    // virtual void setParams(double gamma_1, double gamma_2, double h);
    virtual void setParams(const std::vector<double>& params) override;
    virtual Eigen::Vector2d poseControl(const Eigen::Vector3d& currentState,
                                        const Eigen::Vector3d& desiredState) override;

private:
    // Controller parameters
    double m_gamma_1, m_gamma_2;
    double m_h;
};

#endif
