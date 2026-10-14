# 1.def 函数基本形态
# C++
# int add(int a, int b) 
# {
#    return a + b;
# }
# python 如下：def define
def add(a,b):
    return a + b

# 不写返回值类型 参数也不写类型 
# 情况一：正常填 -> add(1, 2)，返回 3。没问题
# 情况二：填字符串 -> add("hello", "world")，返回 "helloworld"。也没问题！因为 Python 的 + 支持字符串拼接
# 情况三：乱填混合类型 -> add(1, "hello")。程序直接崩溃（报错 TypeError）

# 2.默认参数
def greet(name,greeting = "你好"):  # 规则：有默认值的参数，必须排在没默认值的后面 —— def f(a, b=1) 对，def f(a=1, b) 错
    print(f"{greeting}，{name}")

greet("luoyi")
greet("luoyi","hello")
greet(greeting = "hi" , name = "zhangsan") # 可以清楚的看到自己传入的参数
# 最后一行叫「关键字参数」：按名字传，顺序随便

# 3.返回值 python能一次返回多个
def add(a,b):
    return a + b

def min_max(nums):
    return min(nums),max(nums) # 实际上返回一个元组

low, high = min_max([3,7,12,4,9]) # low, high = (1, 5) 触发元组解包 自动把右边的元组拆开，按顺序分别赋值：low = 1，high = 5
print(low)
print(high)

# 4.第一笔账：不写return会怎样
def add(a,b):
    print(a + b)

result = add(1,2)
print(result) # result的结果是none  Python里所有函数都有返回值 你没写 return，它就自动返回 None

# 5.第二笔账：is None 怎么写 什么时候用
# 如何正确判断一个变量是不是 None，以及为什么要这么写
def find(nums,target):
    for n in nums:
        if n == target:
            return n
# 搜寻 如果在列表 nums 里找到了目标 target，就立刻 return n 结束函数
# 如果循环全部跑完都没有找到，python默认返回None
result = find([1,2,3,4],9)
print(result)

if result is None:
    print("没找到")
# 为什么是 is None 而不是 == None
# is 比较的是“身份”（内存地址） None在python中是一个单例对象 内存里只有一个None
# == 比较的是“值”（内容）不符合规范
# 所以，判断变量是不是 None，永远记住固定写法：
# if result is None:    
# if result is not None:

# 6.作用域
count = 0 # 全局变量

# def increase():
#     count = count + 1 # 报错：UnboundLocalError
    # 在 Python 中，因为这里出现了 `count =`赋值操作，Python 就认定 count 是局部变量。 一定要是count = 即赋值操作，光print(count)不赋值 pyhton会去全局找
    # 然后执行到右边 count + 1 时，去局部找 count，发现还没赋值，所以炸了
    # 只要你在函数里写了 变量名 = 值，这是一个新造的局部变量
    # 规则一句话：函数内「读」外面的变量可以，一「写」就变成局部变量。
# increase()
# 正确解决思路：
count = 0

def increase(current_count):
    return current_count + 1

count = increase(count)
print(count) #1

# 7.import 导入模块
# 三种导入方式
# 第一种 标准导入 import math 把整个 math 模块（工具箱）搬进来。使用里面的工具时，必须加上前缀 math.
import math
print(math.sqrt(16)) #平方根 4.0  math 模块里的数学函数（比如开平方、三角函数、对数等）默认处理的都是浮点数flot 带小数点
print(math.pi) #圆周率 3.141592653589793

# 第二种 精准导入
from math import sqrt # 只导入要用的，之后直接写 sqrt()
print(sqrt(16)) #4.0

# 第三种 别名导入
import math as m # 起个别名
print(m.sqrt(16))

# C++：#include <cmath> 相当于把数学库包含进来，调用时需要用 std::sqrt()， std:: 就像是 Python 里的 math.
# Python：不需要在文件开头写 #include，直接在代码里写 import math 就能用 更加动态和灵活