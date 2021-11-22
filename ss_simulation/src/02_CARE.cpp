/**
 * @file 02_CARE.cpp
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Simple example solving a CARE.
 * @version 1.0
 * @date 2021-06-28
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "ss_algorithm/solver/CARE_Solver.h"

int main(int argc, char* argv[])
{
    // Set input matrices
    Eigen::Matrix2d A, B, Q, R;
    A << 0, 1, 1, -1;
    B << 1, 0, 0, 1;
    Q << 1, 0, 0, 1;
    R << 0.5, 0, 0, 0.5;

    // Set CARE solver
    CARE_Solver solver(A, B, Q, R);
    std::cout << "CARE: A^T*P + P*A - P*B*R^-1*B^T*P + Q = 0" << std::endl
              << "Solution of P is" << std::endl
              << solver.solveCARE() << std::endl;
}