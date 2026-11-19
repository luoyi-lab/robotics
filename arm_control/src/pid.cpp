#include "arm/pid.hpp"
#include <algorithm>
#include <cmath>

namespace arm {

PID::PID(double kp, double ki, double kd, double u_lim, AntiWindup mode)
    : kp_(kp), ki_(ki), kd_(kd), u_lim_(u_lim), mode_(mode) {
    reset();
}

void PID::reset() {
    integral_ = 0.0;
    prev_val_ = 0.0;
    first_ = true;
}

// 注意：函数名前面必须加 PID::
double PID::update(double target, double current, double dt) {
    double e = target - current;
    // 微分先行 & 第一拍不冲击
    double de = first_ ? 0.0 : -(current - prev_val_) / dt;
    prev_val_ = current;
    first_ = false;

    double raw = kp_ * e + ki_ * integral_ + kd_ * de;
    double u = std::max(-u_lim_, std::min(u_lim_, raw));

    // 抗饱和策略
    if (mode_ == AntiWindup::Conditional) {
        if (u == raw) integral_ += e * dt;
    } else if (mode_ == AntiWindup::Clamp) {
        integral_ += e * dt;
        double ilim = u_lim_ / (ki_ > 0 ? ki_ : 1.0);
        integral_ = std::max(-ilim, std::min(ilim, integral_));
    } else {
        integral_ += e * dt;
    }
    return u;
}

} // namespace arm