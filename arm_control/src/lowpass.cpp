#include "arm/lowpass.hpp"

namespace arm {

// alpha = dt / (tau + dt)  —— 这是库的内部实现，调用方不需要知道
LowPassFilter::LowPassFilter(double tau, double dt) {
    // alpha 必须落在 (0, 1]：<=0 不滤波，>1 反馈项变号会振荡
    if (!(tau > 0.0) || !(dt > 0.0)) {
        alpha_ = 1.0;   // 退化成一个安全的"不滤波"，至少不会振
    } else {
        alpha_ = dt / (tau + dt);
    }
}

void LowPassFilter::reset() {
    prev_y_ = 0.0;
    first_ = true;
}

double LowPassFilter::update(double input) {
    if (first_) {
        // 第一拍直通：没有历史数据时，直接把输入当成输出
        // 不这么做的话，第一帧 100.0 会被算成 alpha*100 + (1-alpha)*0 ≈ 9，
        // 机械臂开机瞬间就会被"拉"到一个假位置。
        prev_y_ = input;
        first_ = false;
        return input;
    }
    // 一阶低通公式：y = a*x + (1-a)*y_prev
    double y = alpha_ * input + (1.0 - alpha_) * prev_y_;
    prev_y_ = y;
    return y;
}

} // namespace arm
