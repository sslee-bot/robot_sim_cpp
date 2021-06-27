/**
 * @file StateFeedbackLQR.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Infinite-horizon and continuous-time LQR control based on material:
 * https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator
 * For more advanced LQR control including output feedback LQR, see
 * https://lewisgroup.uta.edu/ee5321/2013%20notes/3%20OP%20feedback%20and%20OPFB%20design%20for%20F16%20lateral%20regulator.pdf
 * @version 1.0
 * @date 2021-06-17
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef OUTPUT_FEEDBACK_LQR_H
#define OUTPUT_FEEDBACK_LQR_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "algorithm/CARE_Solver.h"

class StateFeedbackLQR
{
public:
    StateFeedbackLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C);
    StateFeedbackLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd Q,
                     Eigen::MatrixXd R);
    virtual ~StateFeedbackLQR();
    virtual Eigen::VectorXd generateControlInput(Eigen::VectorXd state);

private:
    virtual void updateFeedbackGain();

    // Dimensions
    unsigned int m_stateDimension;
    unsigned int m_inputDimension;
    unsigned int m_outputDimension;

    // System matrices
    Eigen::MatrixXd m_A, m_B, m_C;

    // Cost matrices
    Eigen::MatrixXd m_Q, m_R;

    // Feedback gain matrix
    Eigen::MatrixXd m_feedbackGainMatrix;
};

#endif