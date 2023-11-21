#ifndef TRANSFORM_IMU_H
#define TRANSFORM_IMU_H

#include <Eigen/Dense>
#include "math_tools.h" // Assuming this file contains necessary mathematical tools

namespace transform_imu {

class InputIMU {
public:
    Eigen::Vector3d acc; // Acceleration
    Eigen::Vector3d omg; // Angular velocity

    // Constructor
    InputIMU(const Eigen::Vector3d& acc, const Eigen::Vector3d& omg)
        : acc(acc), omg(omg) {}
};

Eigen::Vector3d transformVel(const Eigen::Matrix4d& Tba, const Eigen::Vector3d& v, const Eigen::Vector3d& omega);

InputIMU transformIMU(const Eigen::Matrix4d& Tba, const InputIMU& imua, const Eigen::Vector3d& domg);

} // namespace transform_imu

#endif // TRANSFORM_IMU_H
