/**
 * @file StateFeedbackLQR.h
 * @author Sangsu Lee (physism@gmail.com)
 * @brief Infinite-horizon and continuous-time LQR control based on
 * https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator
 * For more advanced LQR control including output feedback LQR, see
 * https://lewisgroup.uta.edu/ee5321/2013%20notes/3%20OP%20feedback%20and%20OPFB%20design%20for%20F16%20lateral%20regulator.pdf
 * @version 1.0
 * @date 2021-06-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef SS_ALGORITHM_STATE_FEEDBACK_LQR_H
#define SS_ALGORITHM_STATE_FEEDBACK_LQR_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "ss_algorithm/API/RobotSimCppGeneral.h"
#include "ss_algorithm/AlgorithmAPI.h"

class StateFeedbackLQR
{
public:
    StateFeedbackLQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C);
    StateFeedbackLQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C,
                     const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    virtual ~StateFeedbackLQR();
    virtual Eigen::VectorXd generateControlInput(const Eigen::VectorXd& state);

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