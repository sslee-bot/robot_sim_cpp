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
    virtual Eigen::VectorXd generateControlInput(Eigen::VectorXd output);

private:
    /**
     * @brief TABLE 8.1-2 Optimal Output Feedback Solution Algorithm
     *
     */
    virtual void updateFeedbackGain();

    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::MatrixXd m_C;
    Eigen::MatrixXd m_feedbackGainMatrix;

    double m_alpha = 0.1;
};

#endif