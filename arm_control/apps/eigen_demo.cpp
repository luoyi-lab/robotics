#include "arm/kinematics.hpp"
#include <cstdio>
#include <cmath>

int main() {
    std::printf("=== arm_control + Eigen 演示 ===\n\n");

    // 两连杆，标准型 D-H
    const Eigen::Matrix4d T1 = arm::dh(0.3, 0.0, 1.0, 0.0);
    const Eigen::Matrix4d T2 = arm::dh(0.7, 0.0, 1.0, 0.0);
    const Eigen::Matrix4d T = T1 * T2;

    std::printf("T = T1 * T2 =\n");
    std::printf(" [%9.6f %9.6f %9.6f %10.6f]\n", T(0,0), T(0,1), T(0,2), T(0,3));
    std::printf(" [%9.6f %9.6f %9.6f %10.6f]\n", T(1,0), T(1,1), T(1,2), T(1,3));
    std::printf(" [%9.6f %9.6f %9.6f %10.6f]\n", T(2,0), T(2,1), T(2,2), T(2,3));
    std::printf(" [%9.6f %9.6f %9.6f %10.6f]\n", T(3,0), T(3,1), T(3,2), T(3,3));

    std::printf("\n末端位置 = (%.6f, %.6f, %.6f)\n", T(0,3), T(1,3), T(2,3));
    std::printf("末端角度 = %.6f rad\n", std::atan2(T(1,0), T(0,0)));

    // 闭式逆 vs 通用逆
    const Eigen::Matrix4d Ti = arm::fast_inverse(T);
    const Eigen::Matrix4d Tis = T.inverse();
    std::printf("\n闭式逆 vs 通用逆误差 = %.3e\n", (Ti - Tis).norm());
    std::printf("T * T^-1 与 I 的误差 = %.3e\n", (T * Ti - Eigen::Matrix4d::Identity()).norm());

    return 0;
}