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

#include "controller/StateFeedbackLQR.h"
#include "model/InvertedPendulum.h"

int main(int argc, char* argv[])
{
    // Set values for simulation
    double initPosition = 0.0;
    double initAngle = 0.1;
    double period = 0.01;

    // Initialize pendulum and controller
    InvertedPendulum pendulum(initPosition, initAngle);
    StateFeedbackLQR controlLQR(pendulum.getMatrixA(), pendulum.getMatrixB(),
                                pendulum.getMatrixC());

    std::cout << "Initial state of the pendulum" << std::endl;
    std::cout << pendulum.stateVector() << std::endl << std::endl;

    for (int i = 0; i < 500; i++) {
        auto state = pendulum.stateVector();
        auto input = (controlLQR.generateControlInput(state))[0];

        pendulum.timeUpdate(input, period);
    }

    std::cout << "State after force applied" << std::endl;
    std::cout << pendulum.stateVector() << std::endl;

    return 0;
}