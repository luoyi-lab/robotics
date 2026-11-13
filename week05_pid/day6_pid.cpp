/* 前几天 在python里面写pid算法 有现成的库帮忙处理一切 今天 要用C++去写真正的底层控制代码 并且引入机器人领域最核心的机制：多线程
真实的机器人里，读取传感器（编码器、IMU）和控制电机（算 PID）通常是两个独立的线程，因为它们需要不同的运行频率。
如果没有多线程保护，机器人数据就会变成一锅粥。

C++核心代码实现：
*/
#include <iostream>
#include <cstdio>     // 用于 std::printf
#include <cmath>
#include <algorithm>  // 用于 std::max, std::min
#include <vector>

class PID {
public:
    // 构造函数，默认 mode = 2 (条件积分)
    PID(double kp, double ki, double kd, double u_lim = 50.0, int mode = 2)
        : kp_(kp), ki_(ki), kd_(kd), u_lim_(u_lim), mode_(mode) {
        reset();
    }

    void reset() {
        integral_ = 0.0;
        prev_val_ = 0.0;
        first_ = true; // 对应 Python 的 prev_val = None
    }

    double update(double target, double current, double dt) {
        double e = target - current;
        
        // 1. 微分先行 & 第一拍不制造冲击
        double de;
        if (first_) {
            de = 0.0;
            first_ = false;
        } else {
            // 只对 current 求导，且带负号
            de = -(current - prev_val_) / dt; 
        }
        prev_val_ = current;

        // 2. 计算理论输出
        double raw = kp_ * e + ki_ * integral_ + kd_ * de;
        
        // 3. 输出限幅
        double u = std::max(-u_lim_, std::min(u_lim_, raw));

        // 4. 抗饱和策略 (mode: 0=无, 1=钳位, 2=条件积分)
        if (mode_ == 0) {
            integral_ += e * dt;
        } else if (mode_ == 1) {
            double ilim = (ki_ > 0) ? (u_lim_ / ki_) : 0.0;
            integral_ += e * dt;
            integral_ = std::max(-ilim, std::min(ilim, integral_));
        } else if (mode_ == 2) {
            if (u == raw) { // 只有没饱和时才积分
                integral_ += e * dt;
            }
        }

        return u;
    }

private:
    double kp_, ki_, kd_, u_lim_;
    int mode_;
    double integral_ = 0.0;
    double prev_val_ = 0.0;
    bool first_ = true;
};

// ==========================================
// 验收标准：一维质量块仿真测试
// ==========================================
int main() {
    // 仿真参数
    const double dt = 0.01;
    const double T  = 20.0;
    const double kp = 10, ki = 5, kd = 5;

    const char* mode_names[] = {"无抗饱和", "钳位", "条件积分"};
    const double lims[]      = {10.0, 20.0};   // ← 两组限幅都要测

    for (double u_lim : lims) {
        std::cout << "=== 限幅 ±" << u_lim << " ===\n";

        for (int mode = 0; mode <= 2; ++mode) {
            PID pid(kp, ki, kd, u_lim, mode);
            double x = 0.0, v = 0.0;
            double max_x = 0.0;

            for (int k = 0; k < static_cast<int>(T / dt); ++k) {
                const double t = k * dt;
                const double target = (t < 5.0) ? 1.0 : 20.0; // 5秒时目标跳变

                const double u = pid.update(target, x, dt);

                // 欧拉积分物理引擎
                v += u * dt;
                x += v * dt;

                if (x > max_x) max_x = x;
            }

            const double overshoot = (max_x - 20.0) / 20.0 * 100.0;
            std::printf("  %-10s 峰值 %8.3f   超调 %6.1f%%\n",
                        mode_names[mode], max_x, overshoot);
        }
        std::cout << "\n";
    }

    return 0;
}

