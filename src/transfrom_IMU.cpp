#include "transfrom_velocity/transfrom_IMU.h"

using namespace math_tools;

namespace transform_imu {

Eigen::Vector3d transformVel(const Eigen::Matrix4d& Tba, const Eigen::Vector3d& v, const Eigen::Vector3d& omega) {
    Eigen::Matrix3d Rba;
    Eigen::Vector3d tba;

    std::tie(Rba, tba) = makeRt(Tba);

    return Rba * v;
}

InputIMU transformIMU(const Eigen::Matrix4d& Tba, const InputIMU& imua, const Eigen::Vector3d& domg) {
    Eigen::Matrix3d Rba;
    Eigen::Vector3d tba;
    std::tie(Rba, tba) = makeRt(Tba); // makeRt関数で回転と平行移動を抽出

    InputIMU imub(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 加速度と角速度の変換
    imub.acc = Rba * imua.acc -
        skew(Rba * imua.omg) * skew(Rba * imua.omg) * tba +
        skew(tba) * Rba * domg;
    imub.omg = Rba * imua.omg;

    return imub;
}

} // namespace transform_imu
