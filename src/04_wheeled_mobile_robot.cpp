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

#include <memory>

#include "controller/WheeledMobileRobotController/Jang2009.h"
#include "controller/WheeledMobileRobotController/Kim2002_1.h"
#include "model/WheeledMobileRobot.h"

int main(int argc, char* argv[])
{
    // Set values for simulation
    double period = 0.01;
    double initX = 0.0, initY = 0.0, initTheta = 0.0, centerToWheelAxis = 0.0;
    double desiredX = 1.0, desiredY = 1.0, desiredTheta = 0.0;
    double gamma_1 = 1.5, gamma_2 = 1.0, h = 2.0;
    double k = 1.0, mu = 3.0;

    // Construct desired state vector
    Eigen::Vector3d desiredState;
    desiredState << desiredX, desiredY, desiredTheta;

    // Initialize wheeled mobile robot and controller
    WheeledMobileRobot robot(initX, initY, initTheta, centerToWheelAxis);
    std::shared_ptr<WheeledMobileRobotController> pController;

    while (true) {
        int controllerCode;
        std::cout << "Select controller" << std::endl << std::endl;

        std::cout << "1. Jang2009" << std::endl;
        std::cout << "2. Kim2002_1" << std::endl;
        std::cout << std::endl;

        std::cout << "Enter number: ";
        std::cin >> controllerCode;

        if (controllerCode == 1) {
            pController = std::make_shared<Jang2009>(gamma_1, gamma_2, h);
            break;
        }
        else if (controllerCode == 2) {
            pController = std::make_shared<Kim2002_1>(k, mu);
            break;
        }
        else {
            std::cout << "Invalid number. Please try again." << std::endl << std::endl;
        }
    }

    std::cout << std::endl;
    std::cout << "Initial state of the wheeled mobile robot" << std::endl;
    std::cout << robot.stateVector() << std::endl << std::endl;

    for (int i = 0; i < 1000; i++) {
        auto currentState = robot.stateVector();
        auto input = pController->poseControl(currentState, desiredState);

        robot.timeUpdate(input, period);
    }

    std::cout << "State after controller applied" << std::endl;
    std::cout << robot.stateVector() << std::endl;

    return 0;
}
