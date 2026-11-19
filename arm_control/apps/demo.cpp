// W6 Day4 演示程序
//   1) 一维质量块仿真：2 种限幅 × 3 种抗饱和 = 6 个数字
//      应与 week05_pid/day6_pid.cpp 一字不差（重构：改结构，不改行为）
//   2) 一阶低通阶跃响应：alpha = 0.090909，看 y 何时达到 0.63
#include <cstdio>

#include "arm/lowpass.hpp"
#include "arm/pid.hpp"

namespace {

constexpr double kDt = 0.01;   // 仿真步长
constexpr double kT  = 20.0;   // 总时长
constexpr double kKp = 10.0;
constexpr double kKi = 5.0;
constexpr double kKd = 5.0;

// 一维质量块：v += u*dt; x += v*dt
void run_pid_case(double u_lim, arm::AntiWindup mode, const char* name) {
    arm::PID pid(kKp, kKi, kKd, u_lim, mode);
    double x = 0.0;
    double v = 0.0;
    double max_x = 0.0;

    const int steps = static_cast<int>(kT / kDt);
    for (int k = 0; k < steps; ++k) {
        const double t = k * kDt;
        const double target = (t < 5.0) ? 1.0 : 20.0;   // 第 5 秒目标跳变

        const double u = pid.update(target, x, kDt);

        v += u * kDt;
        x += v * kDt;
        if (x > max_x) max_x = x;
    }

    const double overshoot = (max_x - 20.0) / 20.0 * 100.0;
    std::printf("  %-10s 峰值 %8.3f   超调 %6.1f%%\n", name, max_x, overshoot);
}

}  // namespace

int main() {
    // ---------- 1) PID：6 个数字 ----------
    const char* names[] = {"无抗饱和", "钳位", "条件积分"};
    const arm::AntiWindup modes[] = {
        arm::AntiWindup::None,
        arm::AntiWindup::Clamp,
        arm::AntiWindup::Conditional,
    };
    const double lims[] = {10.0, 20.0};

    for (double lim : lims) {
        std::printf("=== 限幅 ±%.0f ===\n", lim);
        for (int i = 0; i < 3; ++i) {
            run_pid_case(lim, modes[i], names[i]);
        }
        std::printf("\n");
    }

    // ---------- 2) 低通阶跃响应 ----------
    // 传物理量 tau 和 dt，alpha 由库自己算（不要在这里硬编码 0.090909）
    constexpr double kTau = 0.1; // 时间常数 0.1 s
    arm::LowPassFilter lpf(kTau, kDt);
    std::printf("=== 低通阶跃响应 (tau=%.2fs, dt=%.2fs, alpha=%.6f) ===\n",
                kTau, kDt, lpf.alpha());

    bool reached = false;

    for (int k = 0; k < 51; ++k) {
        const double t = k * kDt;
        // 第 0 拍输入 0 预热（first_ 直通），之后阶跃到 1
        const double input = (k == 0) ? 0.0 : 1.0;
        const double y = lpf.update(input);

        std::printf("  t=%.2f  y=%.4f\n", t, y);

        if (!reached && y >= 0.63) {
            std::printf("  >>> 第一次 y >= 0.63 出现在 t = %.2f s\n", t);
            reached = true;
        }
    }

    if (!reached) {
        std::printf("  >>> 没有达到 0.63，检查 alpha 和 dt\n");
    }

    return 0;
}
