#ifndef TRANSFORM_VELOCITY_H
#define TRANSFORM_VELOCITY_H

#include <Eigen/Dense>

namespace transfrom_velocity {

Eigen::Matrix<double, 6, 1> transformVelocity3D(const Eigen::Matrix4d& Tba, const Eigen::Matrix<double, 6, 1>& va);

Eigen::Vector3d transformVelocity2D(const Eigen::Matrix3d& Tba, const Eigen::Vector3d& vwa);

} // namespace transfrom_velocity

#endif // TRANSFORM_VELOCITY_H
