#include "controller/OutputFeedbackLQR.h"

OutputFeedbackLQR::OutputFeedbackLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C)
    : m_A(A), m_B(B), m_C(C)
{
    updateFeedbackGain();
}

OutputFeedbackLQR::~OutputFeedbackLQR()
{
}

Eigen::VectorXd OutputFeedbackLQR::generateControlInput(Eigen::VectorXd output)
{
    return -m_feedbackGainMatrix * output;
}

void OutputFeedbackLQR::updateFeedbackGain()
{
}
