#include "controller/StateFeedbackLQR.h"

StateFeedbackLQR::StateFeedbackLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C)
    : m_A(A), m_B(B), m_C(C)
{
    // TODO: Check if A is square
    // Initialize dimension values
    m_stateDimension = m_A.cols();
    m_inputDimension = m_B.cols();
    m_outputDimension = m_C.rows();

    // Initialize cost matrices
    m_Q = m_C.transpose() * m_C;
    m_R = Eigen::MatrixXd::Identity(m_inputDimension, m_inputDimension);

    // Initialize feedback gain
    updateFeedbackGain();
}

StateFeedbackLQR::StateFeedbackLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C,
                                   Eigen::MatrixXd Q, Eigen::MatrixXd R)
    : m_A(A), m_B(B), m_C(C), m_Q(Q), m_R(R)
{
    // TODO: Check if A is square
    // Initialize dimension values
    m_stateDimension = m_A.cols();
    m_inputDimension = m_B.cols();
    m_outputDimension = m_C.rows();

    // Initialize feedback gain
    updateFeedbackGain();
}

StateFeedbackLQR::~StateFeedbackLQR()
{
}

Eigen::VectorXd StateFeedbackLQR::generateControlInput(Eigen::VectorXd state)
{
    return -m_feedbackGainMatrix * state;
}

void StateFeedbackLQR::updateFeedbackGain()
{
    CARE_Solver solver(m_A, m_B, m_Q, m_R);
    auto P = solver.solveCARE();

    m_feedbackGainMatrix = m_R.inverse() * m_B.transpose() * P;
}
