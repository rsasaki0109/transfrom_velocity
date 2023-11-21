#include "transfrom_velocity/transfrom_velocity.h"
#include "transfrom_velocity/math_tools.h"

using namespace math_tools;

namespace transfrom_velocity {

Eigen::Matrix<double, 6, 1> transformVelocity3D(const Eigen::Matrix4d& Tba, const Eigen::Matrix<double, 6, 1>& va) {
    Eigen::Matrix3d Rba;
    Eigen::Vector3d tba;
    std::tie(Rba, tba) = makeRt(Tba);

    Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Identity();
    M.block<3, 3>(0, 0) = Rba;
    M.block<3, 3>(0, 3) = skew(tba) * Rba;
    M.block<3, 3>(3, 3) = Rba;

    Eigen::Matrix<double, 6, 1> vb = M * va;
    return vb;
}

Eigen::Vector3d transformVelocity2D(const Eigen::Matrix3d& Tba, const Eigen::Vector3d& vwa) {
    Eigen::Matrix2d Rba;
    Eigen::Vector2d tba;
    std::tie(Rba, tba) = makeRt(Tba);

    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    M.block<2, 2>(0, 0) = Rba;
    Eigen::Vector2d hat_result = hat2d(-tba).col(0);
    M.block<2, 1>(0, 2) = hat_result;

    Eigen::Vector3d vwb = M * vwa;
    return vwb;
}

} // namespace transfrom_velocity
