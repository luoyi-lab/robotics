/* P控制器 叫比例控制器
output（输出） = kp（系数）× error（误差）
很好理解
*/
#include <iostream>
using namespace std;

class PController {
public:
    // 1. explicit 关键字：防止隐式转换 它要求你必须老老实实写 PController c(2.0);。如果不加它，别人瞎写一句 PController c = 2.0; 也能跑
    explicit PController(double kp) : kp_(kp) {}
    // 2. 成员函数尾部加 const：承诺不修改任何成员变量
    double update(double error) const  // 我 update 函数，绝对不会修改类里面的任何数据 尤其是那个 kp_
    {
        // 基本类型（double）按值传递，不用 const double&
        return kp_ * error;  // 把存下来的系数，乘以传入的误差，交出去
    }

    // 获取当前 kp 值（不修改成员，加 const）
    double kp() const { return kp_; }

    // 修改 kp 值（要修改成员，不能加 const）
    void setKp(double kp) { kp_ = kp; }

private:
    // 3. 命名惯例：成员变量加下划线后缀
    double kp_; 
};

int main() {
    // ========== 测试 1：普通对象 ==========
    PController c(2.0); // 创建了一个名字叫c的P控制器 把比例系数定为2.0
    cout << c.update(0.5) << endl;   // 2.0 * 0.5 = 1
    cout << c.update(-1.5) << endl;  // 2.0 * -1.5 = -3
    
    c.setKp(10.0);                   // 修改 kp
    cout << c.update(0.5) << endl;   // 10.0 * 0.5 = 5

    // ========== 测试 2：const 对象 ==========
    // 声明一个常量对象，意味着它本身不能被修改
    const PController cc(3.0); 
    
    // cc.setKp(5.0); // ⚠️ 如果打开这行，会编译报错！因为 const 对象不能调用非 const 函数
    
    cout << cc.update(2.0) << endl;  // 3.0 * 2.0 = 6。能调用，因为 update 后面有 const
    
    return 0;
}

// 核心一句话：如果你有一个 const（只读）的机器人状态数据，你就只准调用那些屁股后面带了 const 的函数
