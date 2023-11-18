#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "transfrom_velocity/transfrom_velocity.h"

int main() {
    Eigen::Matrix4d Tba = Eigen::Matrix4d::Identity();
    double angle = 0;
    Tba.block<3, 3>(0, 0) = (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    Tba.block<3, 1>(0, 3) = Eigen::Vector3d(1, 0, 0);

    Eigen::Matrix<double, 6, 1> va;
    va << 0.5, 0, 0, 0, 0, 1;

    Eigen::Matrix<double, 6, 1> vb = transfrom_velocity::transformVelocity3D(Tba, va);

    std::ofstream file("velocity_data.txt");
    file << "Tba:\n" << Tba << "\n\n";
    file << "Original velocity (va): " << va.transpose() << "\n";
    file << "Transformed velocity (vb): " << vb.transpose() << "\n";
    file.close();

    return 0;
}