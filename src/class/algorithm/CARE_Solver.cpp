#include "algorithm/CARE_Solver.h"

CARE_Solver::CARE_Solver(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R)
    : m_A(A), m_B(B), m_Q(Q), m_R(R)
{
    // TODO: Check if dimensions of matrices are valid
}

CARE_Solver::~CARE_Solver()
{
}

Eigen::MatrixXd CARE_Solver::solveCARE()
{
    // Define Hamiltonian matrix
    Eigen::MatrixXd Z(2 * m_A.rows(), 2 * m_A.rows());
    Z << m_A, -m_B * m_R.inverse() * m_B.transpose(), -m_Q, -m_A.transpose();

    // For decomposition
    Eigen::ComplexSchur<Eigen::MatrixXd> solver(Z);

    auto U = solver.matrixU();
    auto rowsNumOfU = U.rows();
    auto colsNumOfU = U.cols();
    auto U11 = U.topLeftCorner(rowsNumOfU / 2, colsNumOfU / 2);
    auto U21 = U.bottomLeftCorner(rowsNumOfU / 2, colsNumOfU / 2);

    return (U21 * U11.inverse()).real();
}