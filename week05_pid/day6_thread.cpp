/* 多线程的坑（Mutex vs Atomic  互斥锁 vs 原子变量）
真实的机器人为什么要用多线程？
因为传感器的读取频率（比如 1000Hz）和控制算法的执行频率（比如 500Hz）通常不一样。
通常，一个线程专门死循环读传感器，另一个线程专门死循环算 PID。

但如果多个线程同时去修改同一个变量（比如你在两个线程里都让 sum += 1），就会发生数据竞争（Data Race），导致结果完全错误。

多线程是怎么操作的？（C++ 实操）
在 C++ 里，多线程的语法极其简单，核心就三句话：

定义函数：写两个独立的函数，一个叫 read_sensor()，一个叫 run_controller()。
创建线程：在 main 函数里，创建两个 std::thread，把它们绑到这两个函数上。
合并（join）：告诉主程序，“等这俩小弟干完活，咱们再退出”。

三个任务如下：
*/
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <vector>

// ==========================================
// 必备零件：Day 3 翻译过来的 C++ PID 类
// ==========================================
class PID {
public:
    PID(double kp, double ki, double kd, double u_lim = 50.0, int mode = 2)
        : kp_(kp), ki_(ki), kd_(kd), u_lim_(u_lim), mode_(mode) {
        reset();
    }

    void reset() {
        integral_ = 0.0;
        prev_val_ = 0.0;
        first_ = true; 
    }

    double update(double target, double current, double dt) {
        double e = target - current;
        double de;
        if (first_) {
            de = 0.0;
            first_ = false;
        } else {
            de = -(current - prev_val_) / dt; 
        }
        prev_val_ = current;

        double raw = kp_ * e + ki_ * integral_ + kd_ * de;
        double u = std::max(-u_lim_, std::min(u_lim_, raw));

        if (mode_ == 0) {
            integral_ += e * dt;
        } else if (mode_ == 1) {
            double ilim = (ki_ > 0) ? (u_lim_ / ki_) : 0.0;
            integral_ += e * dt;
            integral_ = std::max(-ilim, std::min(ilim, integral_));
        } else if (mode_ == 2) {
            if (u == raw) {
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
// 任务 A：mutex 基本用法
// ==========================================
void task_A_mutex() {
    std::cout << "\n--- 任务 A: Mutex 基本用法 ---\n";
    std::mutex mtx;
    double result = 0.0;

    // 线程 1：每次加 1.0，加 5 次
    std::thread t1([&]{
        for (int i = 0; i < 5; ++i) 
        {
            std::lock_guard<std::mutex> lock(mtx); // 构造加锁，离开作用域析构自动解锁
            result += 1.0;
        }
    });

    // 线程 2：每次加 10.0，加 5 次
    std::thread t2([&]{
        for (int i = 0; i < 5; ++i) 
        {
            std::lock_guard<std::mutex> lock(mtx);
            result += 10.0;
        }
    });

    t1.join();
    t2.join();
    
    // 期望结果：5 * 1.0 + 5 * 10.0 = 55
    std::cout << "期望 55, 实际 result = " << result << "\n";
}


// ==========================================
// 任务 B：atomic vs 裸变量（数据竞争）
// ==========================================
void task_B_atomic_vs_plain() 
{
    std::cout << "\n--- 任务 B: Atomic vs 裸变量 ---\n";
    std::atomic<long long> a{0};
    long long p = 0;
    const int iterations = 2000000;
    const int num_threads = 4;

    // 工作函数：每个线程执行 200 万次 ++
    auto work = [&]() {
        for (int k = 0; k < iterations; ++k) {
            ++a;
            ++p;
        }
    };

    // 启动 4 个线程
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(work);
    }

    // 等待所有线程结束
    for (auto& t : threads) {
        t.join();
    }

    std::cout << "atomic = " << a.load() << "  (每次都精确为 8,000,000)\n";
    std::cout << "plain  = " << p << "  (每次少 60% 左右，且每次不一样)\n";
}


// ==========================================
// 任务 C：传感器线程 + 控制线程（贴机器人场景）
// ==========================================
void task_C_robot_scenario() 
{
    std::cout << "\n--- 任务 C: 传感器与控制线程 ---\n";
    
    // 共享数据，使用 atomic<double> 保证读写安全
    std::atomic<double> sensor_value{0.0};

    // 传感器线程
    std::thread sensor([&]{
        for (int i = 0; i < 1000; ++i) 
        {
            sensor_value.store(std::sin(0.01 * i)); // 存储传感器读数
            // 读取频率：每 100 微秒读取一次 (1000Hz)
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    // 控制线程
    std::thread controller([&]{
        // 注意：PID 对象是局部变量，只有控制线程自己访问，
        // 不需要加锁，非常高效。
        PID pid(80, 30, 3); 
        
        for (int i = 0; i < 500; ++i) 
        {
            // 读取共享的传感器数据
            double v = sensor_value.load();
            
            // 计算控制量。目标设为 0.0
            double u = pid.update(0.0, v, 0.01);
            
            // 现实中这里会发送给电机驱动，这里避免编译器警告
            (void)u; 
            
            // 控制周期：每 2 毫秒计算一次 (500Hz)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    // 等待两个线程结束
    sensor.join();
    controller.join();
    
    std::cout << "任务 C 完成（无打印输出，说明运行正常，无死锁或数据竞争）。\n";
}


// ==========================================
// 主函数：按顺序执行三个任务
// ==========================================
int main() 
{
    task_A_mutex();
    task_B_atomic_vs_plain();
    task_C_robot_scenario();
    return 0;
}