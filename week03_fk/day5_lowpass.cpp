/* 第二部分 一阶低通滤波器
先解释一下什么是一阶低通滤波器：
什么是滤波：真实传感器传回来的数据，都是带有毛刺和噪声的，比如：100, 102, 99, 101, 105, 98，数据上蹿下跳，滤波就是像筛子一样，把毛刺挂掉，让数据变的平滑
什么是低通：低通”就是“让低频信号（变化缓慢的信号）通过，把高频信号（变化剧烈的信号，噪声）拦住
什么是一阶：在数学上，“一阶”指的是这个系统只包含一阶微分（或者说离散化后，当前输出只依赖上一次的输出）
如果叫“二阶”，那不仅要看上一次，还要看上上次（多引入一个电容/积分项）

为什么需要：
真实传感器信号全是噪声：关节编码器、IMU、力传感器 —— 直接拿来做控制，噪声会被放大
低通滤波器的任务：让慢变的信号通过，把快变的噪声滤掉

核心数学：
1.核心公式：  y[k] = α·x[k] + (1-α)·y[k-1]
x[k]              这一拍的输入（含噪声）
y[k-1]            上一拍的输出
y[k]              这一拍的输出
α             0 < α < 1，越小越平滑，但如果真信号变了，你反应极度迟钝（延迟极大） 当α极大时 对新输入的反应很快 但是传感器上的毛刺（噪声）全漏过来了，等于没滤波

通俗来说：我这次算出来的结果（y[k]） = 一小部分（α）这次的新数据（x[k]） + 一大部分（1−α）上次的旧结果（y[k-1]）
所以新数据带来的突变被"磨圆"了

α 和时间常数 τ 的关系：  α = dt / (τ + dt)

dt（采样周期）：你的代码多久跑一次？比如 1000Hz 的控制循环，dt = 0.001 s
τ（时间常数）：信号达到阶跃 63% 所需的时间  比如：我给你一个突变数据，滤波器的输出爬到目标值一半多（63%）需要几秒钟
工程上直接说"我要一个 τ=0.1 秒的滤波器"，比说 α 更直观

今天的类这样写：一个经典的、工业级的一阶低通滤波器
class LoePassFilter
{
public:
    LowassFilter(double tau, double dt): alpha_(dt/(tau + dt)), y_(0.0), initialized_(false){}
    // alpha_(...)：用你传入的 tau 和 dt 算出平滑系数 存进私有变量alpha_  y_(0.0)：初始化上一帧的输出值为 0  initialized_(false)：初始化“是否已启动”的开关为 false

    double update(double x)   接收当前的传感器原始数据 x（比如电机当前角度是 100.5），返回滤波后的平滑数据（比如 100.0）
    {
        if (!initialized_)  刚开机时 initialized_ = false 代表还没有见过任何数据 这时 ！（非）发力 把 !initialized_ 变成true 执行{}里的代码 输出y_  后面再继续输入数据 !initialized_ = false  这个if{}代码将不再运行
        {
            y_ = x;
            initialized_ = true;
            return y_;
        }
        y_ = alpha_ * x + (1.0-alpha_) * y_;
        return y_;
    }

    void reset()   //重置函数  机器人急停重启、传感器拔了重插、或者机械臂断电了。此时必须把 initialized_ 改回 false，把历史数据 y_ 归零  防止旧数据污染新数据
    {
        initialized_ = false;
        y_ = 0.0;
    }

private:
    double alpha_;
    double y_;
    bool initialized_;
};

if (!initialized_)  作用很大：
如果没有它，第一帧数据假如为100直接： 0.1 * 100 + 0.9 * 0 = 10 明明是100 变为10 机器人电机直接崩溃
有了它：第一帧直接把输入当输出，假如第二帧为105：0.1 * 105 + 0.9 * 100 = 100.5 之后才开始滤波。 这叫初始化偏置，真实工程里必须处理
*/

// 代码训练：
#include <iostream>
#include <cmath>
#include <iomanip> // 用于控制输出格式
using namespace std;

class LowPassFilter
{
public:
    LowPassFilter(double tau, double dt): alpha_(dt/(tau + dt)), y_(0.0), initialized_(false){}


    double update(double x)   
    {
        if (!initialized_)  
        {
            y_ = x;
            initialized_ = true;
            return y_;
        }
        y_ = alpha_ * x + (1.0-alpha_) * y_;
        return y_;
    }

    void reset()   
    {
        initialized_ = false;
        y_ = 0.0;
    }

private:
    double alpha_;
    double y_;
    bool initialized_;
};

//验证三件事：
int main()
{
    double dt = 0.01;
    double tau = 0.1;
    // (1)阶跃响应：输入从0突然到1.0
    cout << "---阶跃响应（tau=0.1s）---" <<endl;
    LowPassFilter lpf1(tau, dt);
    lpf1.update(0.0); // 先喂一个 0，让滤波器知道"现在在 0"

    for(int k = 0; k <= 30; k++)   //跑31次循环 因为 dt=0.01 秒，这就代表模拟了 0 到 0.3 秒 的真实物理过程
    {
     double t = k * dt;  // 计算当前的真实物理时间 t
     double y = lpf1.update(1.0); // 突然输入一个恒定值1.0（阶跃信号）

     // 每5拍打印一次，方便观察
     if(k % 5 == 0)
     {
         cout << fixed << setprecision(3) << "t =" << t << "s  y = " << y <<endl;
         // fixed << setprecision(3)：强制让小数保留 3 位
     }

     // 找一下 y 什么时候第一次超过0.63
     if (y >= 0.63 && k > 0)  // k>0 排除初始时刻
     {
         cout << "突破0.63的时间大约是：t = " << t << "s （刚好等于tau）" <<endl;  //这里如果没有lpf1.update(0.0);  一定要把if (!initialized_)  关掉 因为它会把y_强制搞到1.0再开始滤波 这样就会导致输出的t=0.010s 即第一下就达成了y >= 0.63
         break; //找到就退出循环
     }
    }

    // (2)噪声平滑：输入 1.0 + 噪声
    cout << "\n=== ② 噪声平滑验证 ===" << endl;
    LowPassFilter lpf2(tau, dt);
    cout << "对比原输入(波动大) 和 滤波后输出(波动小):" << endl;
    for(int k = 0; k < 10; k++)
    {
         // 用sin函数造一个剧烈的噪声，波动范围在0.5 到1.5 之间
         double noise = 0.5 * sin(k * 1.5);
         double x = 1.0 + noise;
         double y = lpf2.update(x);
 
         cout << fixed << setprecision(3) << "输入 x=" << x << "  ->  输出 y=" << y << endl;
    }

    // (3) 初始化偏置：信号本来就在 100
    cout << "\n=== ③ 初始化偏置验证 ===" << endl;
    LowPassFilter lpf3(tau, dt);

    double first_y = lpf3.update(100.0);
    cout << "真实信号是 100.0 第一拍的输出是：" << first_y << endl;
    if(first_y == 100.0) 
    {
        cout << "-> 完美！没有从 0 开始爬，直接对齐了真实值！" << endl;
    } else {
        cout << "-> 糟糕，出现了初始化爬升！" << endl;   // 把if (!initialized_) 关掉会出现这种情况
    }

    return 0;
}