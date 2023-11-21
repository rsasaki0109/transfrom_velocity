#include <iostream>
#include <Eigen/Dense>
#include "transfrom_velocity/transfrom_IMU.h"

using namespace transform_imu;

// Function to check the equality of two IMU objects
bool checkIMUEquality(const InputIMU& imu1, const InputIMU& imu2, double tolerance = 1e-5) {
    return (imu1.acc - imu2.acc).norm() < tolerance && (imu1.omg - imu2.omg).norm() < tolerance;
}

void testTransformIMU() {
    std::cout << "Testing transformIMU..." << std::endl;

    Eigen::Matrix4d Tba = Eigen::Matrix4d::Identity();
    Tba.topLeftCorner<3, 3>() = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d acc(1.0, 0.0, 0.0);
    Eigen::Vector3d omg(0.0, 0.0, 1.0);
    InputIMU imua(acc, omg);

    Eigen::Vector3d domg(0.1, 0.1, 0.1);

    Eigen::Vector3d expected_acc = Tba.topLeftCorner<3, 3>() * acc;
    Eigen::Vector3d expected_omg = Tba.topLeftCorner<3, 3>() * omg;
    InputIMU expected_output_imu(expected_acc, expected_omg);

    InputIMU output_imu = transformIMU(Tba, imua, domg);

    if (checkIMUEquality(output_imu, expected_output_imu)) {
        std::cout << "Test Passed\n";
    } else {
        std::cout << "Test Failed\n";
    }
}

int main() {
    testTransformIMU();
    return 0;
}
