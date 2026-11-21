#pragma once

#include <Eigen/Dense> // 必须包含，因为类里用到了 Eigen::Matrix4d

namespace arm {

// 标准型 D-H 变换
Eigen::Matrix4d dh(double theta, double d, double a, double alpha);

// 闭式逆（针对齐次矩阵的特殊求逆）
Eigen::Matrix4d fast_inverse(const Eigen::Matrix4d& T);

} // namespace arm