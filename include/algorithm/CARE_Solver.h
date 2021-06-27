/**
 * @file CARE_Solver.h
 * @author Sang Su Lee (physism@gmail.com)
 * @brief Continuous algebraic Riccati equation (CARE) solver implemented based on
 * https://en.wikipedia.org/wiki/Algebraic_Riccati_equation#Solution
 * @version 1.0
 * @date 2021-06-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef CARE_SOLVER_H
#define CARE_SOLVER_H

#include <Eigen/Dense>
#include <iostream>

class CARE_Solver
{
public:
    CARE_Solver(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R);
    virtual ~CARE_Solver();
    virtual Eigen::MatrixXd solveCARE();

private:
    // Matrices
    Eigen::MatrixXd m_A, m_B, m_Q, m_R;
};

#endif