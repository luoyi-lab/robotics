#pragma once // 防止头文件被重复包含

namespace arm {

/**
 * 一阶低通滤波器 (First-Order Low-Pass Filter)
 * 作用：平滑信号，滤除高频噪声。
 * 公式：y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
 *       其中 alpha = dt / (tau + dt)
 */
class LowPassFilter {
public:
    // 构造函数：传【物理量】tau 和 dt，不要传 alpha
    //   tau : 时间常数（秒）—— 阶跃之后输出爬到 63% 所需的时间
    //   dt  : 采样周期（秒）—— 你的控制循环多久跑一次
    //   tau 越大越平滑、但滞后越严重；dt 越小，离散化越接近连续系统。
    //
    //   为什么不让调用方直接传 alpha？
    //     alpha 是【实现细节】。调用方脑子里想的是"我要一个 0.1 秒的滤波器"，
    //     不是"我要 0.0909"。把推导公式留在库里，调用方就不用记它。
    LowPassFilter(double tau, double dt);

    // 核心函数：输入当前测量值，返回滤波后的输出值
    double update(double input);

    // 重置滤波器状态，用于重新开始新的滤波过程
    void reset();

    // 查一下内部算出来的 alpha（调试用，alpha = dt/(tau+dt)）
    double alpha() const { return alpha_; }

private:
    double alpha_;        // 平滑系数，由 (tau, dt) 算出来，不是外部传进来的
    double prev_y_ = 0.0; // 上一次的输出值 (对应公式里的 y[k-1])
    bool first_ = true;   // 第一拍标志位，防止开机时输出突变（给 0 或者随机值）
};

} // namespace arm
