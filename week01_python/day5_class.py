# 类
# 先看熟悉的C++
# class Point
# {
#  public:
#      double x;
#      double y;
#      Point(double x_, double y_) : x(x_), y(y_) {}   // 构造函数 接收两个参数 x_ 和 y_ 
#                                                      x(x_)：意思是“用参数 x_ 的值，去初始化成员变量 x
#      double distance_to(const Point& other) const    //Point& other 用引用的方式接受另一个点坐标
#     {
#       double dx = x - other.x;
#       double dy = y - other.y;
#       return std::sqrt(dx*dx + dy*dy);           //std::sqrt(...) 为开平方根（square root） 
#     }
# };

#int main() 
#{
#    Point p1(0, 0);
#    Point p2(3, 4);
#    std::cout << p1.distance_to(p2);   // 5        std:: 是 C++ 的“命名空间防卫术”。告诉编译器：“我要用的是标准库里的 cout 而不是自己定义的cout
#                 调用p1的distance函数 把p2传进去
#}
# python版本如下：
import math # 导入模块  对照C++：#include <cmath>

class Point:   # 类的定义
    def __init__(self,x,y):   # __init__： Python 的构造函数专用名 前后各两个下划线
        self.x = x            # 这就是 C++  里的 this 指针。但它是显式的
        self.y = y            # self.x = x  python标准的赋值写法 把外部传进来的参数 x，塞给这个对象自己的 self.x 同时self.x 就自动成了这个对象的成员变量

    def distance_to(self,other): # self：代表当前对象（this） other：代表传入的另一个点 Python不需要写 const Point& 
        dx = self.x - other.x  
        dy = self.y - other.y
        return math.sqrt(dx*dx + dy*dy)

p1 = Point(0,0)  # 对比C++：Point p1(0, 0);
p2 = Point(3,4)
print(p1.distance_to(p2))
# 五个关键差异
#                      C++                              Python
# 构造函数       名字和类一样Point()           固定叫__init__ 前后两个下划线
# this               隐式 不用写            必须显式写self 并且定义函数时：第一个参数必须留给 self
# 初始化列表       :x(x_),y(y_)                   直接self.x = x
# 访问控制         pubilc:/private:                 没有 python不强制
# 属性声明       必须在类里写 double x:       不用声明，self.x = x 就直接创建了

# 重点拆解 最后一条“反直觉”的演示（动态属性）
class Foo:
    pass   # 空的 什么都没有

f = Foo()
f.name = "zhangsan"
print(f.name)   #zhangsan
# 在 C++ 里，一个对象占多大内存、有哪些成员变量，什么类型等等，在编译期就已经完全锁死了
# 但在 Python 里，对象本质上是一个动态字典（__dict__）
# 当写 f.name = "abc" 时，Python 默默在 f 内部加了一个键值对：{"name": "abc"} 所以属性可以在运行时凭空长出来

# 今天最核心的东西： self到底是什么
# 直接回答 self就是C++里的this指针 self 就是那个显式化、需要你亲手写在参数里的this
def __init__(self,x,y):
    self.x = x
    self.y = y
# self.x（左边）：代表对象身上的成员变量（相当于 C++ 的 this->x）
# x（右边）：代表传进来的局部参数（相当于 C++ 构造函数里的 x_ 或形参 x）
# self.x = x 完全等价于 C++ 里的：this->x = x_

# 为什么调用时不用传self
p1 = Point(3,5)
# 因为 p1 = Point(3, 5) 时，Python 底层实际干的事是：
# Point.__init__(p1,3,5) 把p1自动塞进self的位置

# 什么时候看出self的作用 
p3 = Point(1,4)
p4 = Point(8,9)

print(p3.x)
print(p4.x)

# 同一个类造出两个对象，各自的x互不干扰 区分当前到底是哪个对象在操作