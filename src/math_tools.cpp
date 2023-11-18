#include "transfrom_velocity/math_tools.h"

namespace transfrom_velocity {

Eigen::Matrix3d v2m(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << std::cos(v(2)), -std::sin(v(2)), v(0),
         std::sin(v(2)),  std::cos(v(2)),  v(1),
         0,               0,               1;
    return m;
}

Eigen::Matrix2d expSO2(double v) {
    Eigen::Matrix2d m;
    m << std::cos(v), -std::sin(v),
         std::sin(v),  std::cos(v);
    return m;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d m;
    m << 0,         -vector(2), vector(1),
         vector(2), 0,          -vector(0),
         -vector(1), vector(0), 0;
    return m;
}

Eigen::Vector3d unskew(const Eigen::Matrix3d& m) {
    return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> makeRt(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    return std::make_pair(R, t);
}

std::pair<Eigen::Matrix2d, Eigen::Vector2d> makeRt(const Eigen::Matrix3d& T) {
    Eigen::Matrix2d R = T.block<2, 2>(0, 0);
    Eigen::Vector2d t = T.block<2, 1>(0, 2);
    return std::make_pair(R, t);
}

Eigen::Matrix2d hat2d(const Eigen::Vector2d& v) {
    Eigen::Matrix2d M;
    M <<  0, -v(1),
          v(0), 0;
    return M;
}

} // namespace transfrom_velocity
