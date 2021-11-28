/**
 * @file 03_inverted_pendulum.cpp
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Inverted pendulum control example.
 * @version 1.0
 * @date 2021-06-28
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <Eigen/Dense>
#include <chrono>
#include <iostream>

#include "ss_algorithm/control/StateFeedbackLQR.h"
#include "ss_model/inverted_pendulum/InvertedPendulum.h"

int main(int argc, char* argv[])
{
    // Set values for simulation
    double initPosition = 0.0;
    double initAngle = 0.1;
    double period = 0.01;
    int iterationNum = 500;

    // Initialize pendulum and controller
    InvertedPendulum pendulum(initPosition, initAngle);
    StateFeedbackLQR controlLQR(pendulum.getMatrixA(), pendulum.getMatrixB(),
                                pendulum.getMatrixC());

    std::cout << std::endl;
    std::cout << "Initial state of the pendulum" << std::endl;
    std::cout << pendulum.stateVector() << std::endl << std::endl;

    // For computation time check
    auto startTime = std::chrono::system_clock::now();

    // Control
    for (int i = 0; i < iterationNum; i++) {
        auto state = pendulum.stateVector();
        auto input = (controlLQR.generateControlInput(state))[0];

        pendulum.timeUpdate(input, period);
    }

    // Computation time
    std::chrono::duration<double> duration = std::chrono::system_clock::now() - startTime;

    std::cout << "State after force applied" << std::endl;
    std::cout << pendulum.stateVector() << std::endl << std::endl;
    std::cout << "Average computation time: " << duration.count() / iterationNum << std::endl;

    return 0;
}