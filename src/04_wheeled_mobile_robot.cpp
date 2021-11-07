/**
 * @file 04_wheeled_mobile_robot.cpp
 * @author Sang Su Lee (physism@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-11-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "model/WheeledMobileRobot.h"

int main(int argc, char* argv[])
{
    // Set values for simulation
    double initX = 0.0;
    double initY = 0.0;
    double initTheta = 0.0;
    double centerToWheelAxis = 0.0;
    // double period = 0.01;

    // Initialize wheeled mobile robot
    WheeledMobileRobot robot(initX, initY, initTheta, centerToWheelAxis);

    std::cout << "Initial state of the wheeled mobile robot" << std::endl;
    std::cout << robot.stateVector() << std::endl << std::endl;

    for (int i = 0; i < 500; i++) {
        // TODO: implement controller for robot
        // auto state = robot.stateVector();
        // auto input =

        // robot.timeUpdate(input, period);
    }

    std::cout << "State after controller applied" << std::endl;
    std::cout << robot.stateVector() << std::endl;

    return 0;
}
