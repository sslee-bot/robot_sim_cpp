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
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "ss_algorithm/estimator/EstimatorAPI.h"

using namespace std;

int main(int argc, char* argv[])
{
    // Set values for simulation
    double period = 0.1;
    double x = 0.0, y = 0.0;
    double stddev_a = 0.25, stddev_z = 1.2;

    // System matrices
    Eigen::Matrix2d A;
    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 1, 2> C;
    Eigen::Matrix2d Q;
    Eigen::Matrix<double, 1, 1> R;
    A << 1.0, period, 0.0, 1.0;
    B << 0.5 * pow(period, 2), period;
    C << 1.0, 0.0;
    Q << 0.25 * pow(period, 4), 0.5 * pow(period, 3), 0.5 * pow(period, 3), pow(period, 2);
    Q *= pow(stddev_a, 2);
    R << pow(stddev_z, 2);

    // State
    Eigen::Vector2d state;
    state << x, y;

    // Error covariance
    Eigen::Matrix2d P;
    P.setIdentity();

    // Kalman filter
    DiscreteLinearKF filter(A, B, C, P, Q, R);

    // Simulation properties
    double t = 0.0;
    double finishTime = 100.0;
    int count = 0;
    Eigen::Matrix<double, 1, 1> z;  // Position measurement
    Eigen::Matrix<double, 1, 1> a;  // Control input (acceleration)
    z.setZero();
    a << 0.2;

    // For saving results
    double realPos, noise;
    vector<double> vPlotX, vPlotY1, vPlotY2;

    // For random number (noise) generation
    unsigned int seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    normal_distribution<double> distribution(0.0, stddev_z);

    // For computation time check
    auto startTime = std::chrono::system_clock::now();

    // Simulation
    bool isFinish = false;
    while (!isFinish) {
        // Measurement
        realPos = 0.1 * (pow(t, 2) - t);
        noise = distribution(generator);
        z[0] = realPos + noise;

        // Filtering
        filter.predict(a);
        filter.correct(z);
        state = filter.getState();

        // Save result
        vPlotX.push_back(t);
        vPlotY1.push_back(realPos);
        vPlotY2.push_back(state[0]);

        // Update time
        t += period;
        if (t > finishTime) {
            isFinish = true;
        }

        count++;
    }

    // Computation time
    std::chrono::duration<double> duration = std::chrono::system_clock::now() - startTime;
    std::cout << "Average computation time: " << duration.count() / count << std::endl;

    // RMSE
    double error, rmse, sum = 0;
    for (int i = 0; i < count; i++) {
        error = vPlotY2[i] - vPlotY1[i];
        sum += std::pow(error, 2);
    }
    rmse = std::sqrt(sum / count);
    std::cout << "RMSE: " << rmse << std::endl;

    // // Plot result
    // auto axes = CvPlot::makePlotAxes();
    // axes.create<CvPlot::Series>(vPlotX, vPlotY1, "k-");
    // axes.create<CvPlot::Series>(vPlotX, vPlotY2, "o");
    // CvPlot::show("05_1D_object_tracking", axes);

    return 0;
}
