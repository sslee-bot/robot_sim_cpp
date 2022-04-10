#include "ss_algorithm/estimator/DiscreteLinearKF.h"

DiscreteLinearKF::DiscreteLinearKF(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                   const Eigen::MatrixXd& C, const Eigen::MatrixXd& P,
                                   const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
    : m_A(A), m_B(B), m_C(C), m_P(P), m_Q(Q), m_R(R)
{
    // TODO: Check if A is square
    // Initialize dimension values
    m_stateDimension = m_A.cols();
    m_inputDimension = m_B.cols();
    m_outputDimension = m_C.rows();

    // Initialize Kalman gain
    m_K.noalias() = m_P * m_C.transpose() * (m_C * m_P * m_C.transpose() + m_R).inverse();

    // Initialize indentity matix
    m_I.setIdentity(m_stateDimension, m_stateDimension);

    // Initialize state
    m_state.setZero(m_stateDimension);
}

DiscreteLinearKF::~DiscreteLinearKF(void)
{
}

void DiscreteLinearKF::setInitialState(const Eigen::MatrixXd& initialState)
{
    if (initialState.size() != m_stateDimension) {
        std::cout << "Invalid size of input initial state: " << initialState.size() << std::endl;
        return;
    }

    m_state = initialState;
}

void DiscreteLinearKF::predict(const Eigen::VectorXd& controlInput)
{
    m_state = m_A * m_state + m_B * controlInput;
    m_P = m_A * m_P * m_A.transpose() + m_Q;
}

void DiscreteLinearKF::correct(const Eigen::VectorXd& measurement)
{
    m_K.noalias() = m_P * m_C.transpose() * (m_C * m_P * m_C.transpose() + m_R).inverse();
    m_state += m_K * (measurement - m_C * m_state);
    m_P = (m_I - m_K * m_C) * m_P;
}
