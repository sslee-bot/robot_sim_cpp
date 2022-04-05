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

#define CVPLOT_HEADER_ONLY

#include <CvPlot/cvplot.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <random>
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    // Set values for simulation
    double period = 0.1;
    double x = 0.0, y = 0.0;
    double stddev_a = 0.25, stddev_z = 1.2;

    // System
    Eigen::Matrix2d A;
    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 1, 2> H;
    Eigen::Matrix2d Q;
    Eigen::Matrix<double, 1, 1> R;

    A << 1.0, period, 0.0, 1.0;
    B << 0.5 * pow(period, 2), period;
    H << 1.0, 0.0;
    Q << 0.25 * pow(period, 4), 0.5 * pow(period, 3), 0.5 * pow(period, 3), pow(period, 2);
    Q *= pow(stddev_a, 2);
    R << pow(stddev_z, 2);

    // State
    Eigen::Vector2d state;
    state << x, y;

    // Error covariance
    Eigen::Matrix2d P;
    P.setIdentity();

    // Simulation
    double t = 0.0;
    double finishTime = 100.0;
    double z;  // position measurement
    Eigen::Matrix<double, 1, 1> a;
    a << 0.2;
    Eigen::Matrix2d I;
    I.setIdentity();
    Eigen::Matrix<double, 2, 1> K;

    vector<double> vPlotX, vPlotY1, vPlotY2;

    double realPos, noise;
    unsigned int seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    normal_distribution<double> distribution(0.0, stddev_z);

    bool isFinish = false;
    while (!isFinish) {
        // Measurement
        realPos = 0.1 * (pow(t, 2) - t);
        noise = distribution(generator);
        z = realPos + noise;

        // Prediction
        state = A * state + B * a;
        P = A * P * A.transpose() + Q;

        // Correction
        K.noalias() = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        state = state + K * (z - H * state);
        P = (I - K * H) * P;

        // Save result
        vPlotX.push_back(t);
        vPlotY1.push_back(realPos);
        vPlotY2.push_back(state[0]);

        // Update time
        t += period;
        if (t > finishTime) {
            isFinish = true;
        }
    }

    // Plot result
    auto axes = CvPlot::makePlotAxes();
    axes.create<CvPlot::Series>(vPlotX, vPlotY1, "k-");
    axes.create<CvPlot::Series>(vPlotX, vPlotY2, "o");
    CvPlot::show("05_1D_object_tracking", axes);

    return 0;
}
