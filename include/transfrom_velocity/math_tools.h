#ifndef MATH_TOOLS_H
#define MATH_TOOLS_H

#include <functional>
#include <vector>
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

Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega);

Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);

Eigen::MatrixXd numericalDerivative(
    std::function<Eigen::VectorXd(const std::vector<Eigen::VectorXd>&)> func,
    std::vector<Eigen::VectorXd> params,
    int idx,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> plus = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a + b; },
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> minus = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return a - b; },
    double delta = 1e-5
);

} // namespace transfrom_velocity

#endif // MATH_TOOLS_H