# 1.旋转矩阵的意义（详见机器人学这本书 这里我就直接会用就行）
# 用了矩阵：
# 1.表示旋转：一个矩阵 R 就能代表一个旋转姿态
# 2.复合旋转：直接做矩阵乘法！R_总 = R_y @ R_x 
# 3.点移动：把点 p 变成 p'，直接 p' = R @ p
# 4.求逆（反向旋转）：极其简单，直接 R.T（转置） 因为旋转矩阵是正交矩阵 转置等于逆 而且计算机算转置比算逆快的多
# 一句话总结：矩阵把“复杂的三角函数几何问题”，变成了简单的线性代数问题

# 2.2D 旋转矩阵（简单推导见机器人学书面笔记）
# R(θ) = [[cosθ, -sinθ],
#        [sinθ,  cosθ]]

# 3.三个必须记住的性质
# （1）R.T == R^-1
# （2）R.T @ R == I  #旋转矩阵是正交阵 
# （3）det(R) = 1 #行列式代表这个矩阵对面积或体积的缩放比例 det = 1表明面积或体积完全没变，保持了原样 机器人学中旋转矩阵的作用是“只改变方向，不改变长度和形状

# 4.右手定则
# 2D：逆时针是正方向
# 3D：绕某个轴转的正方向 —— 右手大拇指指向轴的正方向，四指弯曲的方向就是正转

# 5.代码训练
import numpy as np

# (1) 定义函数 参数传的是弧度    Python 的 np.cos() 和 np.sin() 只认弧度     360° = 2pi （弧度）
def rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s],
                     [s, c]])

# (2) 看看转90度的矩阵是啥样
R = rot2d(np.pi/2) # 输入NumPy提供的内置常量 pi/2对应90°
print("R =\n", R) # \n是换行符 
# 应该是 [[0, -1], 
#        [1, 0]]

# (3) 验证正交 R.T = R^-1    R.T @ R = I
print(R.T @ R)

# (4) 验证 det == 1
print(np.linalg.det(R)) # np.linalg：NumPy的线性代数模块

# (5) 验证 R.T 就是逆
print(R.T)
print(np.linalg.inv(R))
# 验证两者是否相等 用np.allclose(对象1 , 对象2)
print(np.allclose(R.T , np.linalg.inv(R)))    # True

# (6) 关键验证：x 轴转 90° 应该到 y 轴
p = np.array([1.0,0.0])
print(R @ p)

# (7) 旋转前后长度必须不变
print("转前长度 =", np.linalg.norm(p)) # norm：在数学里叫“范数”，在几何里就是向量的长度，即模长
print("转后长度 =", np.linalg.norm(R @ p))

# (8) 再试几个角度
R1 = rot2d(np.pi)
R2 = rot2d(np.pi/3)
print(R1 @ p)
print(R2 @ p)



    