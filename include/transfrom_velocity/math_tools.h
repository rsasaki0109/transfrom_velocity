#ifndef MATH_TOOLS_H
#define MATH_TOOLS_H

#include <Eigen/Dense>

namespace transfrom_velocity {

Eigen::Matrix3d v2m(const Eigen::Vector3d& v);

Eigen::Matrix2d expSO2(double v);

double logSO2(const Eigen::Matrix2d& m);

Eigen::Matrix3d skew(const Eigen::Vector3d& vector);

Eigen::Vector3d unskew(const Eigen::Matrix3d& m);

std::pair<Eigen::Matrix3d, Eigen::Vector3d> makeRt(const Eigen::Matrix4d& T);

std::pair<Eigen::Matrix2d, Eigen::Vector2d> makeRt(const Eigen::Matrix3d& T);

Eigen::Matrix2d hat2d(const Eigen::Vector2d& v);

} // namespace transfrom_velocity

#endif // MATH_TOOLS_H