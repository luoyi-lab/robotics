# 1.np.linalg——线性代数
import numpy as np
A = np.array([[1,2],
              [3,4]])

print(np.linalg.det(A)) # -2.0 行列式(1×4 - 2×3)    C++ 对照：Eigen 里写 A.determinant()
print(np.linalg.inv(A)) # 逆矩阵 矩阵乘以它的逆，结果等于单位矩阵    C++ 对照：Eigen 里写 A.inverse()
# 为什么重要：W2 的齐次变换矩阵求逆 —— 机械臂的坐标系反变换，全靠它
# 什么是奇异矩阵 即行列式 = 0 矩阵是不可逆的 这在机器人里很常见 比如当机械臂伸直时 雅可比就不可逆 
# 此时再强行用 inv() 程序会直接崩溃 报 LinAlgError
# 遇到了“奇异”怎么办 W3学到：
# 1：伪逆Pseudo-inverse  即使矩阵不可逆，Python 也能帮你求出一种“最接近”的解
# 2：阻尼最小二乘（Damped Least Squares） 在数学上故意加一点微小的扰动（阻尼），让原本等于0的行列式变成非0，从而强行求出一个稳定的逆
 
B = np.linalg.inv(A)
print(A * B) # 经典坑 * 代表是逐元素相乘 而不是矩阵乘法
print(A @ B) # 这个才是标准的矩阵乘法 

# 2.广播机制
# 先看效果
a = np.array([[1,2,3],
              [4,5,6]])  # shape(2,3)

b = np.array([10,20,30]) # shape(3,)因为b是一维的 或者可以写成shape(1,3) 但一定要知道它是一维的
#二维应该这样写 b = np.array([[10,20,30]])
print(a + b)
# 简单来说，广播就是：
# 当两个形状不同的数组进行运算时，NumPy 会“自动复制/拉伸”较小的数组，直到它们形状匹配，然后再逐元素运算
# 从最后一个维度（最右边）往左比，每一个维度满足以下两个条件之一
# 1.相等
# 2.其中一个是1（可以被拉伸复制）

c = np.array([1])
print(a + c)

# 反过来
d = np.array([[10],
              [20]])  # shape(2,1)
print(a + d)