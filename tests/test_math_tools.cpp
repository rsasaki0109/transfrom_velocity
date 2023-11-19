#include <iostream>
#include <Eigen/Dense>
#include "transfrom_velocity/math_tools.h"

using namespace transfrom_velocity;

void checkEquality(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b, double tolerance = 1e-5) {
    if ((a - b).norm() < tolerance) {
        std::cout << "OK\n";
    } else {
        std::cout << "NG\n";
    }
}

// void testHSO3() {
//     std::cout << "Testing HSO3..." << std::endl;

//     Eigen::Vector3d x(0.5, 0.6, 0.7);
//     Eigen::Vector3d dx(0.02, 0.03, 0.03);

//     Eigen::Matrix3d R1 = expSO3(x + dx);
//     Eigen::Matrix3d R2 = expSO3(x) * expSO3(HSO3(x) * dx);

//     checkEquality(R1, R2);
// }

// void testSO3() {
//     std::cout << "Testing SO3..." << std::endl;

//     Eigen::Vector3d v(1.0, 0.3, 2.0);
//     Eigen::Matrix3d R = expSO3(v);
//     Eigen::Matrix3d R2 = expSO3(logSO3(R));
//     Eigen::Matrix3d R3 = expSO3(logSO3(R));

//     checkEquality(R, R2);
//     checkEquality(R2, R3);
// }

// void testSE3() {
//     std::cout << "Testing SE3..." << std::endl;

//     Eigen::VectorXd v(6);
//     v << 1.0, 0.3, 2.0, 1.0, -3.2, 0.2;  // 6-element vector for SE3
//     Eigen::Matrix4d T = expSE3(v);
//     Eigen::Matrix4d T2 = expSE3(logSE3(T));
//     Eigen::Matrix4d T3 = expSE3(logSE3(T));

//     checkEquality(T, T2, 1e-5);
//     checkEquality(T2, T3, 1e-5);
// }

void testNumericalDerivative() {
    std::cout << "Testing Numerical Derivative of SO3..." << std::endl;

    auto residual = [](const std::vector<Eigen::VectorXd>& params) -> Eigen::VectorXd {
        Eigen::Vector3d x = params[0];
        Eigen::Vector3d a = params[1];
        Eigen::Matrix3d R = expSO3(x);
        return (R * a).eval();  // 3-dimensional rotated vector
    };

    auto plus = [](const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) -> Eigen::VectorXd {
        return logSO3(expSO3(x1) * expSO3(x2));
    };

    Eigen::Vector3d x(0.5, 0.2, 0.2);  // example rotation vector
    Eigen::Vector3d a(1.0, 2.0, 3.0);  // example vector

    // Theoretical Jacobian calculation
    Eigen::Matrix3d R = expSO3(x);
    Eigen::Matrix3d J = -R * skew(a);

    Eigen::MatrixXd J_numerical = numericalDerivative(residual, {x, a}, 0, plus);

    double error = (J - J_numerical).norm();
    std::cout << "Error: " << error << std::endl;
    if (error < 1e-4) {
        std::cout << "Numerical derivative test passed." << std::endl;
    } else {
        std::cout << "Numerical derivative test failed." << std::endl;
    }
}

int main() {
    // testHSO3();
    // testSO3();
    // testSE3();
    testNumericalDerivative();

    std::cout << "All tests completed." << std::endl;
    return 0;
}