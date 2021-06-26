#include "controller/OutputFeedbackLQR.h"

OutputFeedbackLQR::OutputFeedbackLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C)
    : m_A(A), m_B(B), m_C(C)
{
    // TODO: Check if A is square
    // Initialize dimension values
    m_stateDimension = m_A.cols();
    m_inputDimension = m_B.cols();
    m_outputDimension = m_C.rows();

    // Initialize cost matrices
    m_Q = Eigen::MatrixXd::Identity(m_stateDimension, m_stateDimension);
    m_R = Eigen::MatrixXd::Identity(m_inputDimension, m_inputDimension);

    // Initialize feedback gain
    updateFeedbackGain();
}

OutputFeedbackLQR::~OutputFeedbackLQR()
{
}

void OutputFeedbackLQR::setQ(Eigen::MatrixXd Q)
{
    // TODO: Check input matrix is valid
    m_Q = Q;
}

void OutputFeedbackLQR::setR(Eigen::MatrixXd R)
{
    // TODO: Check input matrix is valid
    m_R = R;
}

Eigen::VectorXd OutputFeedbackLQR::generateControlInput(Eigen::VectorXd output)
{
    return -m_feedbackGainMatrix * output;
}

void OutputFeedbackLQR::updateFeedbackGain()
{
    // Hamiltonian matrix
    unsigned int sizeOfZ = 2 * m_stateDimension;
    Eigen::MatrixXd Z(sizeOfZ, sizeOfZ);
    Z << m_A, -m_B * m_R.inverse() * m_B.transpose(), -m_Q, -m_A.transpose();

    // m_feedbackGainMatrix =
}
