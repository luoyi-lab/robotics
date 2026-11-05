/* C++ 虚函数 多态 虚析构函数

一.核心概念
1.多态 一句话解释就是同一个接口 不同的实现 （类似于同一个插口可以接风扇 也可以接取暖器）
比如你有一个“基类指针”，它既可以指向 PController 对象，也可以指向 PIDController 对象，调用同名函数 update() 时，它们各自执行各自的动作

2.虚函数 (virtual)：告诉编译器，这个函数是“多态”的。要允许子类重写它，并且必须通过虚表（vtable）动态决定调用谁的版本。

3.override 关键字：在子类里写上它，等于告诉编译器：
“我确定我要重写父类的虚函数，如果你发现拼写错误或者参数不匹配，请直接报错骂我”。这是极好的编程习惯。

4.纯虚函数 (= 0)：如果基类里的函数写成 virtual void update() = 0;，
那这个基类就变成了“抽象类”。你不能直接 new Controller()，必须写个子类继承它并实现这个函数。

5.虚析构函数 (virtual ~Controller())：最致命的坑！ 如果你用基类指针指向子类对象，在 delete 时，
如果析构函数不是虚函数，它只会调用基类的析构函数，子类的析构函数根本不会执行，导致子类里 new 出来的内存泄漏。

二.底层原理（虚表 vtable 是什么）
当你给类写了虚函数，C++ 编译器在编译时，会偷偷给这个类建一张“表”（vtable），里面存满了指向具体函数的指针。
同时，这个类的每个对象，内存里都会多藏一个不可见的指针 vptr，指向这张表。

当执行 base->update() 时，程序不会直接硬编码去调用 Controller::update。而是：

1.找到 base 对象的 vptr。
2.顺着 vptr 查到对应的 vtable。
3.在表里找到 update 这个坑位，跳过去执行。
因为如果 base 实际上指向的是 PIDController，它的 vptr 就是指向 PIDController 的虚表。所以就能完美地“动态绑定”到子类的实现上。

*/
#include <iostream>
#include <vector>
#include <string>

using namespace std;

// ==========================================
// 1. 基类定义
// ==========================================
class Controller 
{
public:
    // 构造函数：为了演示，打印一下
    Controller() { cout << "  [基类] Controller 构造" << endl; }
    
    // 虚析构函数（必须加 virtual！）
    // 实验时，你可以尝试把这里的 virtual 删掉，看看最后会发生什么
    virtual ~Controller() { cout << "  [基类] Controller 析构" << endl; }

    // 纯虚函数，使得 Controller 成为抽象类
    // 子类必须实现这个函数
    virtual void update(double err) = 0; // 这里是统一接口 
    // 意思是所有子公司都必须有个叫 update 的功能，输入一个误差数字 err 进去，具体怎么搞我不管
};

// ==========================================
// 2. 子类：P 控制器
// ==========================================
class PController : public Controller // 开个子公司叫 P，继承了总公司的所有ViewController
{ 
private:
    // 故意申请一块堆内存，用来演示如果不写虚析构函数，这块内存就会泄漏
    double* p_data; 
public:
    PController() 
    {
        p_data = new double[100]; // 申请内存
        cout << "  [子类] PController 构造" << endl;
    }
    
    // override 是一张保证书：“我 P 公司发誓，我准确无误地实现了总公司的 update 规范
    void update(double err) override //这里就是写不同的实现
    {
        // 写 P 公司的具体算法：误差乘 10 输出
        cout << "  执行 P 控制器: 误差 = " << err << "，输出 = " << err * 10.0 << endl;
    }

    // 子类的析构函数
    ~PController() override 
    {
        delete[] p_data; // 释放内存
        cout << "  [子类] PController 析构 (内存已释放)" << endl;
    }
};

// ==========================================
// 3. 子类：PID 控制器
// ==========================================
class PIDController : public Controller 
{
public:
    PIDController() { cout << "  [子类] PIDController 构造" << endl; }
    ~PIDController() override { cout << "  [子类] PIDController 析构" << endl; }

    void update(double err) override {
        cout << "  执行 PID 控制器: 误差 = " << err << "，输出 = " << err * 50.0 << endl;
    }
};

// ==========================================
// 4. 主程序：多态调用与内存测试
// ==========================================
int main() {
    cout << "===== 开始测试多态 =====" << endl;
    
    // 核心：用一个基类指针数组，装下不同的子类对象
    vector<Controller*> controllers;
    controllers.push_back(new PController());
    controllers.push_back(new PIDController());

    // 循环调用，这就是“多态”的魅力：不用关心具体是什么控制器，统一调 update
    cout << "\n--- 循环遍历执行 update ---" << endl;
    for (Controller* c : controllers) {
        c->update(1.5); // 这里会自动根据实际对象类型，调用对应的版本
    }

    // 释放内存
    cout << "\n--- 开始释放内存 ---" << endl;
    for (Controller* c : controllers) {
        delete c; // 如果用到了虚析构函数，这里会正确地先调子类析构，再调基类析构
    }

    // ==========================================
    // 附加：反例测试（怎么制造内存泄漏）
    // ==========================================
    cout << "\n===== 反例：非虚析构函数导致的内存泄漏 =====" << endl;
    cout << "（注意：如果在上面的代码里删掉基类的 virtual ~Controller()，下面的 delete 会只调基类析构，子类的 p_data 永远泄漏）" << endl;
    
    return 0;
}

/*
一句话总结它们的作用：
虚函数（virtual）：立规矩，定下“必须重写的空壳子”。

多态：老板省事，喊一个口令，下面自动执行对应动作。

虚表（vtable）：背后的查号台，保证程序找对执行的人。

虚析构（virtual ~）：拆迁前一定要做的安全检查，防止子类的资源泄露。

和机器人的关系：
W6 把 W3–W5 的代码整理成库时，控制器会写成「一个基类 + 多个子类」：
Controller (抽象基类)
  ├── PController
  ├── PIDController
  └── MPCController
主循环里只持有一个 Controller*，换算法不用改主循环一行。
*/