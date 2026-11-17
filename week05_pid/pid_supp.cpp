/* C++ pid实现完整代码
*/
#include <iostream>
#include <cstdio>     // 用于 std::printf  用于格式化输出一行结果（对齐、小数位）
#include <cmath>
#include <algorithm>  // 用于 std::max, std::min
#include <vector>

class PID
{
public:
    // 构造函数，默认mode = 2（条件积分）  冒号后面的成员初始化列表：对象创建时，直接把参数的值“塞”给类的成员变量
    PID(double kp, double ki, double kd, double u_lim = 50.0, int mode = 2): kp_(kp), ki_(ki), kd_(kd), u_lim_(u_lim), mode_(mode)
    {
         reset();
    }

    void reset()
    {
        integral_ = 0.0;  // 把积分清零
        prev_val_ = 0.0;  // 把上一拍的测量值清零
        first_ = true;  // 对应 Python 的 prev_val = None  把首拍标志设为 true  意思现在是第一拍了
    }

    double update(double target, double current, double dt)
    {
        double e = target - current;

        // 1.微分先行 & 第一拍不制造冲击
        double de;
        if(first_)  // 第一拍：de = 0
        {
            de = 0.0;
            first_ = false;  // 第一拍结束了 把 first_ 置 false
        }
        else
        {
            // 只对current求导 且带负号 因为 e=target−current 
            de = -(current - prev_val_)/dt;
        }
        prev_val_ = current;  // 更新历史值，供下一拍使用

        // 2.计算理论输出
        double raw = kp_ * e + ki_ * integral_ + kd_ * de;

        // 3.输出限幅
        double u = std::max(-u_lim_, std::min(u_lim_, raw));  // -u_lim_ 限制下限  std::min(u_lim_, raw) 限制上限

        // 4.抗饱和策略（mode: 0=无， 1=钳位， 2=条件积分）这里是人为选择抗饱和策略的
        if(mode_ == 0)
        {
            integral_ += e * dt;
        }
        else if(mode_ == 1)
        {
            /*三目运算符
            条件 (ki_ > 0)：判断你的积分系数 ki_ 是不是大于 0
            结果 A (u_lim_ / ki_)：如果大于 0，就算出 u_lim_ / ki_ 作为上限。
            结果 B 0.0：如果 ki_ 小于等于 0，上限直接给 0.0*/
            double ilim = (ki_ > 0)?(u_lim_/ki_) : 0.0;
            integral_ += e * dt;
            integral_ = std::max(-ilim, std::min(ilim, integral_));  // 不管积分怎么累积，都不能超出 [-ilim, ilim] 这个范围
        }
        else if(mode_ == 2)
        {
            if(u == raw)  // 只有没饱和时才积分
            {
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

// 验收标准：一维质量块仿真测试
int main()
{
    // 仿真参数
    const double dt = 0.01;
    const double T = 20.0;
    const double kp = 10, ki = 5, kd = 5;

    const char* mode_names[] = {"无抗饱和", "钳位", "条件积分"};
    const double lims[] = {10.0, 20.0};  // 两组限幅

    for(double u_lim : lims)  // 每次从 lims 里拿出一个元素，就叫它 u_lim
    {
        std::cout << "=== 限幅 ±" << u_lim << " ===\n";

        for(int mode = 0; mode <= 2; ++mode)
        {
            PID pid(kp, ki, kd, u_lim, mode);
            double x = 0.0, v = 0.0;
            double max_x = 0.0;

            for(int k = 0; k < static_cast<int>(T / dt); ++k)  // 时间步循环，共 2000 步
            {
                const double t = k * dt;
                const double target = (t < 5.0) ? 1.0 : 20.0; // 5秒时目标跳变

                const double u = pid.update(target, x, dt);

                // 欧拉积分物理引擎
                v += u * dt;
                x += v * dt;

                if(x > max_x) max_x = x;
            }

            const double overshoot = (max_x - 20.0) / 20.0 * 100.0;  // 超调
            std::printf("  %-10s 峰值 %8.3f   超调 %6.1f%%\n",
                        mode_names[mode], max_x, overshoot);
        }
        std::cout << "\n";
    }
    return 0;
}