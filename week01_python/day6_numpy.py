# NumPy Numerical Python 专门用来做数值分析计算的Python库
# 为什么需要NumPy list 能做列表，但做不了矩阵，也不适合做大量数值计算。
# 而且：1.list里面可以塞入不同类型的元素 很杂乱
#      2.Python 里，list1 + list2 是拼接，而不是对应元素相加
# NumPy 给的是 ndarray（N维数组对象）：同类型、定长、连续内存的多维数组

# 最重要的是：机器人学里的所有东西 —— 旋转矩阵、齐次变换、雅可比、状态向量、MPC 的 QP —— 在 Python 里全部是用 NumPy 表达的
import numpy as np # 导入 numpy 这个库，并给它起个外号叫 np

# 1.创建
np.array([1,2,3])  # 一维
np.array([[1,2,3],
          [4,5,6]]) # 二维
np.zeros((2,3)) # 2×3 全 0
np.ones((2,3)) # 2×3 全 1
np.eye(3) #3×3 单位矩阵（斜对角全为1）
np.arange(0,10,2) #[0 2 4 6 8] 和range一样 但返回数组
# 注意：p.zeros((2, 3))：向 np.zeros 函数传入一个参数，这个参数是一个元组 (2, 3)。它的意思是：“帮我创建一个 2行3列 的全零矩阵
#      如果输入 np.zeros(2, 3)：向 np.zeros 函数传入两个参数，第一个是 2，第二个是 3。这就报错了

# 2.shape、dtype、ndim、size
a = np.array([[1,2,3],
              [4,5,6]])
print(a.shape) # 输出一个(2,3)的元组 描述这个矩阵有多大 即2行3列 这里也可以看出来这个矩阵是2维的

print(a.ndim) # 2  几维 千万不能误认为是几行 
# a 是一个 2维数组（2D Array），它有两层嵌套。
# 第一层：行（axis 0）
# 第二层：列（axis 1）
# 即使矩阵是5行3列 它的ndim依然是2 因为任然是行列两个维度描述的数组
# 3维数组（比如一张彩色的图片，有高、宽、颜色通道）
a3 = np.zeros((28, 28, 3))
print("a3 shape:", a3.shape) # 输出 (28, 28, 3)
print("a3 ndim:", a3.ndim)   # 输出 3     —— 3维数组

print(a.size) # 6  总共几个元素
print(a.dtype) #int64 元素类型   int：整型 64：占64位8字节    NumPy是不能混装元素类型的 

# dtype（date type）是NumPy和list最大的区别
My_list = [1,"abc",True]  # list可以随便混装 这里用My_list 而不是list 因为变量名不能命名为内置类型
arr = np.array([1,2,3.5]) #NumPy只能装一个类型 所以这里会把 1 2 整数升级为浮点
print(arr.dtype) #float 64  整数自动变为浮点
# NumPy 数组里所有元素必须同类型。 这就是它比 list 快得多的原因 —— 同类型 + 连续内存，才能用 C 的速度批量算
# 和 C++ 对照：np.array 更像 std::vector<double>（类型已经确定下来了 必须同类型），list 更像 Python 自己的什么都能塞的容器

# 3.索引和切片
a = np.array([[1,2,3],
             [4,5,6]])

print(a[0]) # 第0行
print(a[0,0]) # 第0行 第0列
print(a[1,2]) # 第1行 第2列

# 取列要用:  冒号的意思是「这一维全要」 切记是“维”
print(a[:,0]) # 所有行（行这个维全都要） 第0列
print(a[:,1]) # 所有行 第1列
print(a[0,:]) # 所有列 第0行 和print(a[0]) 一个意思

# 布尔索引——list做不到的事情
b = np.array([10,20,30,40,50])

print(b > 25) #False False  True  True  True]  b > 25 会逐元素比较 返回的不是一个 True/False，而是一个布尔数组
print(b[b>25]) #把 True 对应位置的数据留下来，把 False 的丢掉 
# 非常关键 以后筛选数据经常用到

# 4.逐元素运算——和list完全不同的地方
# list的 + 是拼接
list1 = [1,2,3]
list2 = [4,5,6]
print(list1 +list2) #[1,2,3,4,5,6]
print(list1*2) #[1,2,3,1,2,3]

# NumPy的 + 是“每个元素各自算”
a = np.array([1,2,3])
b = np.array([4,5,6])
print(a + b) # [5,7,9]
print(a * 2) #[2,4,6]
print(a ** 2) #平方
print(a * b) #相乘 元素相乘
print(b % a) #取余
print(a @ b) #矩阵乘法 1×4 + 2×5 + 3×6 = 32
print(a > 2) #逐元素比较
# 把极其繁琐的重载和循环，全部封装到了底层 C 语言里，并在 Python 层面自动化了

# 5.矩阵乘法：@ 和 * 的区别 非常重要
A = np.array([[1,2],
             [3,4]])

B = np.array([[5,6],
             [7,8]])

print(A * B) # 逐元素相乘
print(A @ B) # 线性代数乘法 第一行×第一列 第一行×第二列 以此类推。。。 1×5 + 2×7 = 19

# 6.转置 .T
A = np.array([[1,2,3],
              [4,5,6]])
print(A.shape) # (2,3)
print(A.T)      # 转置
print(A.T.shape) # (3,2)