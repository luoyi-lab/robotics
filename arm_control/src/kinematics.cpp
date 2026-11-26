#include "arm/kinematics.hpp"
#include <cmath>

namespace arm {

Eigen::Matrix4d dh(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T = (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())
         * Eigen::Translation3d(0, 0, d)
         * Eigen::Translation3d(a, 0, 0)
         * Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX())).matrix();
    return T;
}

Eigen::Matrix4d fast_inverse(const Eigen::Matrix4d& T) {
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d p = T.block<3, 1>(0, 3);
    
    T_inv.block<3, 3>(0, 0) = R.transpose();
    T_inv.block<3, 1>(0, 3) = -R.transpose() * p;
    return T_inv;
}

Eigen::Matrix4d fk2link(double theta1, double theta2, double L1, double L2) {
    return dh(theta1, 0.0, L1, 0.0) * dh(theta2, 0.0, L2, 0.0);
}

} // namespace arm