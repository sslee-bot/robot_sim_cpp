/**
 * @file LinearKF.h
 * @author Sangsu Lee (physism@gmail.com)
 * @brief Discrete-time linear Kalman filter
 * https://en.wikipedia.org/wiki/Kalman_filter
 * @version 1.0
 * @date 2022-04-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SS_ALGORITHM_DISCRETE_LINEAR_KF_H
#define SS_ALGORITHM_DISCRETE_LINEAR_KF_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "ss_algorithm/API/RobotSimCppGeneral.h"

class DiscreteLinearKF
{
public:
    DiscreteLinearKF(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C,
                     const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    virtual ~DiscreteLinearKF(void);
    virtual void setInitialState(const Eigen::MatrixXd& initialState);
    virtual void predict(const Eigen::VectorXd& controlInput);
    virtual void correct(const Eigen::VectorXd& measurement);
    virtual Eigen::MatrixXd getState(void);

private:
    // Dimensions
    unsigned int m_stateDimension, m_inputDimension, m_outputDimension;

    // System matrices
    Eigen::MatrixXd m_A, m_B, m_C;

    // Covariance matrices
    Eigen::MatrixXd m_P, m_Q, m_R;

    // Kalman gain
    Eigen::MatrixXd m_K;

    // Identity matrix
    Eigen::MatrixXd m_I;

    // State
    Eigen::VectorXd m_state;
};

#endif
