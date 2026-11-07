#include <iostream>
using namespace std;

class PController {
public:
    // 1. explicit 关键字：防止隐式转换
    explicit PController(double kp) : kp_(kp) {}

    // 2. 成员函数尾部加 const：承诺不修改任何成员变量
    double update(double error) const {
        // 图片1强调：基本类型（double）按值传递，不用 const double&
        return kp_ * error; 
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
    PController c(2.0);
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
