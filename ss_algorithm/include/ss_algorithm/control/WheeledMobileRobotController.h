/**
 * @file WheeledMobileRobotController.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Abstact class for wheeled mobile robot controller
 * @version 1.0
 * @date 2021-11-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef WHEELED_MOBILE_ROBOT_CONTROLLER_H
#define WHEELED_MOBILE_ROBOT_CONTROLLER_H

#include <Eigen/Dense>

class WheeledMobileRobotController
{
public:
    WheeledMobileRobotController(void)
    {
    }

    virtual ~WheeledMobileRobotController(void)
    {
    }

    virtual Eigen::Vector2d poseControl(const Eigen::Vector3d& currentState,
                                        const Eigen::Vector3d& desiredState) = 0;
};

#endif
