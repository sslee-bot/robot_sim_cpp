/**
 * @file CARE_Solver.h
 * @author Sangsu Lee (physism@gmail.com)
 * @brief Continuous algebraic Riccati equation (CARE) solver implemented based on
 * https://en.wikipedia.org/wiki/Algebraic_Riccati_equation#Solution
 * @version 1.0
 * @date 2021-06-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef SS_ALGORITHM_CARE_SOLVER_H
#define SS_ALGORITHM_CARE_SOLVER_H

#include <Eigen/Dense>
#include <iostream>

#include "ss_algorithm/API/RobotSimCppGeneral.h"

class CARE_Solver
{
public:
    CARE_Solver(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& R);
    virtual ~CARE_Solver();
    virtual Eigen::MatrixXd solveCARE();

private:
    // Matrices
    Eigen::MatrixXd m_A, m_B, m_Q, m_R;
};

#endif