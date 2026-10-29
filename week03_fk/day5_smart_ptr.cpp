/* 第一部分 智能指针
现在写链表是这样的：
ListNode* head = new ListNode(1);  申请
...
delete n;                          手动删
存在三个问题：
1.忘了delete                          会导致内存泄漏
2.中途return或抛异常                  delete执行不到 发生泄漏
3.所有权不清晰                  谁该delete 两个指针指向一块，删两次会崩（参考深浅拷贝）
*/

/*
三种智能指针：
(1) unique_ptr  独占所有权（默认就该用它）   独占指针
#include<memory>
不写 using namespace std                std：标准大仓库   :: 作用域解析运算符 意思是去里面找

auto p = std::make_unique<Point>(3, 5);    // 在堆区安全地创建一个坐标为 (3, 5) 的 Point 对象，并交由变量 p（独占智能指针）管理
p->x;                                      // 访问p所指向的Point对象里面的x属性 
std::cout << p->distance_to_origin();      // p 指向的Point对象去执行它自带的 distance_to_origin() 成员函数，算出距离

不能拷贝 比如：
auto q = p;

只能移动 即所有权转让：
auto q = std::move(p);    现在所有权转移到q p变成nullptr q拥有了p之前指向的那个 Point(3, 5) 对象的所有权

特点：
  同一个时刻只有一个unique_ptr拥有这个对象
  离开作用域自动delete    作用域就是一对大括号 {} 包起来的地盘。比如一个函数内部，或者一个 if 语句内部
  零额外开销（和裸指针一样大）

(2) shared_ptr 共享所有权（引用计数）     共享指针
auto sp = std::make_shared<Point>(1, 2);
auto sp2 = sp;           // 引用计数+1  这里是真正的拷贝

std::cout << sp.use_count();  // 就是查看当前共享个数：2  或者sp2.use_count();  结果是一样的

只有最后一个shared_ptr被销毁时，对象才被delete
什么时候用：多个对象真的需要同时持有同一个东西 比如多个控制器共享一个机器人模型 你不可能给每个控制器复制一份单独的模型 这样内存会爆炸
代价：引用操作时原子操作 比unique_ptr 要慢

(3) weak_ptr  不增加引用次数    弱引用指针
auto sp = str::make_shared<Point>(1, 2);
std::weak_ptr<Point> wp = sp;   // 引用计数还是1 用弱指针看着它，好比隔着橱窗看蛋糕 不买光看

if(auto locked = wp.lock())   // 尝试拿到shared_ptr     lock() 魔法：因为 weak_ptr 只是“看着”，它不知道对象什么时候会被别人删掉。所以在用它之前，必须调用 lock() 试试看 括号里的内容上两行已经给过了
{
  //如果对象还在，lock() 会临时变出一个 shared_ptr（临时让计数加 1，保证你用的这几行代码里对象绝对不被销毁）。
  //用完这个 if 语句后，临时指针销毁，计数减 1
}
else
{
  //如果对象已经被销毁了，lock() 会返回一个空指针（nullptr），走进 else
}

唯一用途：破循环引用
C++ 里极其著名的“互相锁死”死局：
struct A { std::shared_ptr<B> b; };   //结构体 A 内部包含一个指向 B 的共享智能指针
struct B { std::shared_ptr<A> a; };   //结构体 B 内部包含一个指向 A 的共享智能指针
shared_ptr铁律：只要还有一个指针指着这个对象，这个对象就绝对不会被delete 当且仅当引用计数变成 0 时，系统才会来“拆房子”
这就会导致AB互相指着 引用计数永远 ≥1 永远无法释放
怎么解决：改一下：
struct B { std::weak_ptr<A> a; };  
物理画面：A 依然拿着 B 的真实钥匙（强引用），但 B 手里只拿了一张 A 的照片（弱引用）。
当外部销毁时，A 的计数正常归零，A 被销毁。A 销毁时，它手里的 B 的钥匙也跟着消失，B 的计数也归零，B 也被销毁。死局完美破解
*/

/*核心概念：RAII
Resource Acquisition Is Initialization —— 资源获取即初始化

翻译成人话：把资源交给对象的生命周期管理。
1. 拆解这句话的两半：
“资源”：你 new 出来的那块内存（如果不手动 delete 就会泄漏）。或者是你打开的文件、建立网络连接。
“对象”：栈上的那个局部变量 p（它本质上是一个 unique_ptr 类的实例）。
“生命周期”：局部变量 p 在这个大括号 {} 里出生，到大括号 } 结束的时候死亡。

2. 到底是怎么“交给它管理”的？
当你写下 auto p = ... 时，你把内存的“门牌号”塞给了 p。这就叫“资源交给了对象”。
C++ 编译器有一个死规矩：栈上的对象 p，只要离开它所在的 {}，必须立刻销毁。
p 在销毁的最后一瞬间（调用析构函数时），会极其负责任地帮你去执行 delete，把内存还给系统。

你把资源绑在了这个对象的身上，这个对象的出生和死亡，就代表了资源的获取和释放

为什么要叫“资源获取即初始化”：
以前没有 RAII 的“脱裤子放屁”写法：
你先创建一个对象（初始化），然后再去申请资源（获取）。这两件事是分开的
// C语言/老式C++的坏习惯
FILE* file = NULL; // 1. 初始化（声明一个空指针）
file = fopen("data.txt", "r"); // 2. 资源获取（打开文件）
// 中间隔了十万八千里。如果在这中间代码报错了，file 没被关掉，就泄漏了。

有了 RAII 的“天作之合”写法：
为什么不在对象“出生（初始化）”的那一瞬间，就把“资源获取”的事情一起干了呢？
// 现代 C++ 写法
auto p = std::make_unique<Point>(1, 2); 
// 这一行代码里，同时发生两件事：
// 1. 初始化：在堆区 new 了一个 Point(1, 2) 对象。
// 2. 资源获取：把这个对象的地址交给了智能指针 p。

小例子：
void foo() {
    auto p = std::make_unique<Point>(1, 2);
    if (something_wrong) return;      // ← 即使这里 return，p 也会被自动 delete
    throw std::runtime_error("x");    // ← 即使抛异常，也会自动 delete
}
这就是为什么机器人项目里到处是智能指针 —— 控制循环里任何地方抛异常，都不能让资源泄漏
智能指针在机器人项目里面很关键 要重点掌握
*/

// 代码训练：
#include <iostream>
#include <memory>
#include <cmath> 
using namespace std; //这里要注意 真实工程代码最好不用using namespace std;

struct Point
{
    double x, y;
    Point(double x_, double y_):x(x_), y(y_)  // 用初始化列表 x(x_), y(y_) 把传进来的值赋给成员变量
    {

    }
    double norm() const   // const 保证这个函数只读不写
    {
        return sqrt(x*x +y*y);
    }
};

int main()
{
    // (1) unique_ptr
    auto p = make_unique<Point> (3, 4);
    cout << "p->norm() =" << p->norm() <<endl; //5

    auto q = std::move(p);
    cout << "p现在是空吗：" << boolalpha << (p==nullptr) <<endl; 
    
    // (2) shared_ptr
    auto sp1 = make_shared<Point> (1, 2);
    auto sp2 = sp1;
    cout << "引用计数：" << sp1.use_count() <<endl;

    sp2.reset();   // 主动退出：sp2.reset() 意思是“我 sp2 不玩了，把钥匙扔了”。
    cout << "reset后：" << sp1.use_count() <<endl;  // 1

    // (3) weak_ptr
    weak_ptr<Point> wp = sp1;
    cout << "weak不影响计数：" << sp1.use_count() <<endl;  // 1
    if(auto locked = wp.lock())  // 试图拜访：wp.lock() 试图借一张临时门票（变成 shared_ptr）
    {
        cout << "对象还活着：" << locked->norm() <<endl;
    }

    sp1.reset();   // 对象被彻底销毁 再次拜访：wp.lock() 发现房子没了，返回 nullptr
    cout << "对象销毁后 lock 成功吗？" << (wp.lock() == nullptr) <<endl;

    return 0;
}