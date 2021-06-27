/**
 * @file OutputFeedbackLQR.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Output feedback LQR control based on following material.
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

class OutputFeedbackLQR
{
public:
    OutputFeedbackLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C);
    virtual ~OutputFeedbackLQR();
    virtual void setQ(Eigen::MatrixXd Q);
    virtual void setR(Eigen::MatrixXd R);
    virtual Eigen::VectorXd generateControlInput(Eigen::VectorXd output);

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