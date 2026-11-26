#pragma once

#include <Eigen/Dense> // 必须包含，因为类里用到了 Eigen::Matrix4d

namespace arm {

// 标准型 D-H 变换
Eigen::Matrix4d dh(double theta, double d, double a, double alpha);

// 闭式逆（针对齐次矩阵的特殊求逆）
Eigen::Matrix4d fast_inverse(const Eigen::Matrix4d& T);

// 两连杆平面臂正运动学：末端 4x4 齐次变换
//   theta1, theta2 : 关节角（弧度）
//   L1, L2         : 连杆长度
// 约定：标准型 D-H，T = dh(theta1, 0, L1, 0) * dh(theta2, 0, L2, 0)
Eigen::Matrix4d fk2link(double theta1, double theta2, double L1, double L2);
} // namespace arm