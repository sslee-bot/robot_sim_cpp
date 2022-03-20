/**
 * @file 05_1D_object_tracking.cpp
 * @author Sang Su Lee (physism@gmail.com)
 * @brief C++ version of object tracking example
 * https://machinelearningspace.com/object-tracking-python/
 * @version 1.0
 * @date 2022-03-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <Eigen/Dense>
#include <iostream>

// TODO: Header for gaussian random noise

int main(int argc, char* argv[])
{
    // Set values for simulation
    double period = 0.1;
    double x = 0.0, y = 0.0;
    // TODO: Covariances

    // Vector and matrices
    Eigen::Vector2d state;
    state << x, y;
    // TODO: system matrices (A, B)

    // TODO: simulation
    // while (true) {
    // }

    // TODO: Plot result

    return 0;
}
