/**
 * @file 01_matrix_basic.cpp
 * @author Sangsu Lee (physism@gmail.com)
 * @brief Basic matrix operations.
 * @version 1.0
 * @date 2021-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <Eigen/Dense>
#include <iostream>

int main(int argc, char* argv[])
{
    // Matrix whose dimensions are decided at compile time
    Eigen::Matrix2d A;
    A << 1, 2, 3, 4;

    std::cout << std::endl;
    std::cout << "A is" << std::endl << A << std::endl << std::endl;

    // Matrix with dynamic size
    Eigen::MatrixXd B(2, 2);
    B(0, 0) = 3;
    B(1, 0) = 2.5;
    B(0, 1) = -1;
    B(1, 1) = B(1, 0) + B(0, 1);

    std::cout << "B is" << std::endl << B << std::endl << std::endl;

    // Value-initialized vector
    Eigen::Vector2d v1(3, 7);
    Eigen::Vector2d v2(1, 2);

    std::cout << "v1 is" << std::endl << v1 << std::endl << std::endl;

    // Some operations
    std::cout << "A*B:" << std::endl << A * B << std::endl << std::endl;
    std::cout << "A*v1:" << std::endl << A * v1 << std::endl << std::endl;
    std::cout << "Dot product (v1 and v2):" << std::endl << v1.dot(v2) << std::endl << std::endl;
    std::cout << "Cross product of (1, 2, 3) and (2, 4, 5):" << std::endl
              << Eigen::Vector3d(1, 2, 3).cross(Eigen::Vector3d(2, 4, 5)) << std::endl
              << std::endl;
    std::cout << "Trace of A:" << std::endl << A.trace() << std::endl << std::endl;

    return 0;
}
