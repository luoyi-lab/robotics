/* 默写C++ PID 
必须包含这 5 个机制：
	机制	                一句话
1	三项相加	      kp*e + ki*I + kd*de
2	微分先行	      de = -(current - prev_val) / dt，不是对误差求导
3	第一拍不冲	      第一次调用时 de = 0
4	输出限幅	      clamp(raw, ±u_lim)
5	条件积分抗饱和	  只有 u == raw（没饱和）时才累加积分
*/
#include <iostream>  // 负责实现程序的输入和输出
#include <algorithm>  // algorithm：算法 调用处理数据的现成工具，比如排序、查找、反转、求最大值/最小值
#include <cmath>  // 调用各种高级数学运算，比如三角函数（sin, cos）、开根号（sqrt）、绝对值（fabs）等等

class PID
{
public:
    PID(double kp, double ki, double kd, double u_lim = 50.0)
    : kp_(kp), ki_(ki), kd_(kd), u_lim_(u_lim)
    {
        reset();
    }

    void reset()
    {
        integral_ = 0.0;
        prev_val_ = 0.0;
        first_ = true;
    }

    double update(double target, double current, double dt)
    {
        double e = target - current;

        // 微分先行以及第一拍不冲击
        double de = first_ ? 0.0 : -(current - prev_val_) / dt;  // 三元运算符 条件 ? 结果A : 结果B。意思是：如果 first_ 为真，de 就是 0.0；否则，de 就是-(current - prev_val_) / dt
        prev_val_ = current;
        first_ = false;

        // 三项相加
        double raw = kp_ * e + ki_ * integral_ + kd_ * de;

        // 输出限幅
        double u = std::max(-u_lim_, std::min(u_lim_, raw));

        // 条件积分抗饱和
        if(u == raw)
        {
            integral_ += e * dt;
        }

        return u;
    }

private:
    double kp_, ki_, kd_, u_lim_;
    double integral_ = 0.0;
    double prev_val_ = 0.0;
    bool first_ = true;
};

int main()
{
    // 参数设置：限幅 20，条件积分，kp=10, ki=5, kd=5, dt=0.01, 跑 20s，5s 时目标 1->20
    PID pid(10.0, 5.0, 5.0, 20.0);
    double x = 0.0, v = 0.0, dt = 0.01, max_x = 0.0;

    for(int k = 0; k < 2000; ++k)
    {
        double t = k * dt;
        double target = (t < 5.0) ? 1.0 : 20.0;  // 三目运算符 在t=5.0s 时 目标从1突变为20
        double u = pid.update(target, x, dt);
        v += u * dt;
        x += v * dt;
        if(x > max_x) max_x = x;
    }
    std::printf("Peak: %.3f (Expected: 21.806)\n", max_x);
    
    return 0;
}