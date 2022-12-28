/**
 * @file WheeledMobileRobotController.h
 * @author Sangsu Lee (physism@gmail.com)
 * @brief Abstact class for wheeled mobile robot controller
 * @version 1.0
 * @date 2021-11-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef SS_ALGORITHM_WHEELED_MOBILE_ROBOT_CONTROLLER_H
#define SS_ALGORITHM_WHEELED_MOBILE_ROBOT_CONTROLLER_H

#include <Eigen/Dense>
#include <vector>

class WheeledMobileRobotController
{
public:
    WheeledMobileRobotController() = default;
    virtual ~WheeledMobileRobotController() = default;

    virtual void setParams(const std::vector<double>& params) = 0;
    virtual Eigen::Vector2d poseControl(const Eigen::Vector3d& currentState,
                                        const Eigen::Vector3d& desiredState) = 0;
};

#endif
