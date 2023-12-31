#include "transfrom_velocity/math_tools.h"

namespace math_tools {

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

Eigen::Matrix4d makeT(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    Eigen::Matrix3d W = skew(omega); // 'skew' 関数は omega を歪対称行列に変換する関数

    if (theta < 1e-5) {
        return Eigen::Matrix3d::Identity() + W;
    } else {
        Eigen::Matrix3d K = W / theta;
        Eigen::Matrix3d KK = K * K;
        double sin_theta = std::sin(theta);
        double one_minus_cos = 1 - std::cos(theta);
        return Eigen::Matrix3d::Identity() + sin_theta * K + one_minus_cos * KK;
    }
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
    double tr = R.trace();
    Eigen::Vector3d omega;
    Eigen::Vector3d v = Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

    if (tr + 1.0 < 1e-3) {
        // rotation is almost 180 degrees
        if (R(2, 2) > R(1, 1) && R(2, 2) > R(0, 0)) {
            // R(2, 2) is the largest
            Eigen::Vector3d q = Eigen::Vector3d(R(0, 2), R(1, 2), R(2, 2) + 1).normalized();
            omega = M_PI * q;
        } else if (R(1, 1) > R(0, 0)) {
            // R(1, 1) is the largest
            Eigen::Vector3d q = Eigen::Vector3d(R(0, 1), R(1, 1) + 1, R(2, 1)).normalized();
            omega = M_PI * q;
        } else {
            // R(0, 0) is the largest
            Eigen::Vector3d q = Eigen::Vector3d(R(0, 0) + 1, R(1, 0), R(2, 0)).normalized();
            omega = M_PI * q;
        }
    } else {
        double magnitude = 0;
        double tr_3 = tr - 3.0;
        if (tr_3 < -1e-6) {
            double theta = std::acos((tr - 1.0) / 2.0);
            magnitude = theta / (2.0 * std::sin(theta));
        } else {
            magnitude = 0.5 - tr_3 / 12.0 + tr_3 * tr_3 / 60.0;
        }
        omega = magnitude * v;
    }

    return omega;
}

Eigen::Matrix4d expSE3(const Eigen::VectorXd& x) {
    Eigen::Vector3d omega = x.segment<3>(3);
    Eigen::Vector3d v = x.segment<3>(0);
    Eigen::Matrix3d R = expSO3(omega);
    double theta2 = omega.dot(omega);
    if (theta2 > 1e-6) {
        Eigen::Vector3d t_parallel = omega * omega.dot(v);
        Eigen::Vector3d omega_cross_v = omega.cross(v);
        Eigen::Vector3d t = (omega_cross_v - R * omega_cross_v + t_parallel) / theta2;
        return makeT(R, t);
    } else {
        return makeT(R, v);
    }
}

Eigen::VectorXd logSE3(const Eigen::Matrix4d& T) {
    Eigen::Vector3d omega = logSO3(T.block<3, 3>(0, 0));
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    double theta = omega.norm();
    if (theta < 1e-10) {
        return Eigen::VectorXd::Zero(6);
    } else {
        Eigen::Matrix3d W = skew(omega / theta);
        double tan = std::tan(0.5 * theta);
        Eigen::Vector3d WT = W * t;
        Eigen::Vector3d u = t - 0.5 * theta * WT + (1 - theta / (2.0 * tan)) * (W * WT);
        Eigen::VectorXd result(6);
        result << u, omega;
        return result;
    }
}

Eigen::MatrixXd numericalDerivative(
    std::function<Eigen::VectorXd(const std::vector<Eigen::VectorXd>&)> func,
    std::vector<Eigen::VectorXd> params,
    int idx,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> plus,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> minus,
    double delta
) {
    Eigen::VectorXd r = func(params);
    int m = r.size();
    int n = params[idx].size();
    Eigen::MatrixXd J(m, n);

    for (int j = 0; j < n; ++j) {
        Eigen::VectorXd dx = Eigen::VectorXd::Zero(n);
        dx(j) = delta;
        std::vector<Eigen::VectorXd> param_delta = params;
        param_delta[idx] = plus(params[idx], dx);
        J.col(j) = minus(func(param_delta), r) / delta;
    }

    return J;
}

Eigen::Matrix3d HSO3(const Eigen::Vector3d& omega) {
    double theta2 = omega.dot(omega);
    double theta = std::sqrt(theta2);
    bool near_zero = theta2 <= 1e-6;
    Eigen::Matrix3d W = skew(omega);
    if (near_zero) {
        return Eigen::Matrix3d::Identity() - 0.5 * W;
    } else {
        Eigen::Matrix3d K = W / theta;
        Eigen::Matrix3d KK = K * K;
        double sin_theta = std::sin(theta);
        double s2 = std::sin(theta / 2.0);
        double one_minus_cos = 2.0 * s2 * s2;  // [1 - cos(theta)]
        double a = one_minus_cos / theta;
        double b = 1.0 - sin_theta / theta;
        return Eigen::Matrix3d::Identity() - a * K + b * KK;
    }
}

} // namespace transfrom_velocity
