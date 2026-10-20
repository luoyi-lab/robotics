# 今天主要演示旋转的复合
# 1.旋转复合的两套规则 （详见机器人学）
# 先绕X轴转 再绕Y轴转：
# （1）定坐标系 外旋/fixed   始终绕着定坐标系的轴转 写法：R = Ry() @ Rx()   即左乘 与旋转顺序相反
# （2）动坐标系 内旋/body   绕着自己的坐标系转 即轴会跟着物体移动 写法：R = Rx() @ Ry() 即右乘 与旋转顺序相同

# 2.顺序不可交换
# 例如：
# Ry(90°) @ Rx(90°)  ≠  Rx(90°) @ Ry(90°) 很简单的道理 自己画个坐标系转一下就明白了
# 同轴旋转可以交换 这是很自然的
# Rx(90°) @ Rx(30°)  ==  Rx(30°) @ Rx(90°) 

# 3.再重复一次定理：
# 同样是R = Rx() @ Ry() @ Rz() 可以是：
# （1）绕定坐标系转：先绕z轴转 再绕y轴转 再绕x轴转
# （2）绕动坐标系转（即自身的坐标系）：先绕x轴转 再绕y轴转 再绕z轴转

# 4.实际用在哪里
# URFD里面的rpy用的是定坐标系:
# <origin rpy="0 0 1.57" .../>
# 含义：先绕固定 X 转 roll，再绕固定 Y 转 pitch，再绕固定 Z 转 yaw
# R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

# 5.代码训练
import numpy as np
import itertools

# 三个基本旋转
def Rx(theta):
    c , s = np.cos(theta) , np.sin(theta)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]])

def Ry(theta):
    c , s = np.cos(theta) , np.sin(theta)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s,0, c]])

def Rz(theta):
    c , s = np.cos(theta) , np.sin(theta)
    return np.array([[c,-s, 0],
                     [s, c, 0],
                     [0, 0, 1]])

# 定义几根轴与简称
z = np.array([0.0, 0.0, 1.0])
p = x = np.array([1.0, 0.0, 0.0]) 
y = np.array([0.0, 1.0, 0.0])
Rx90,Ry90,Rz90 = Rx(np.pi/2), Ry(np.pi/2), Rz(np.pi/2)


# 1.定坐标系 vs 动坐标系：都叫先绕x轴转 再绕y轴转
R_fixed = Ry90 @ Rx90 # 定坐标系：先绕世界不动的坐标系转 左乘 顺序相反
R_body = Rx90 @ Ry90 # 动坐标系：绕自身在动的坐标系转 右乘 顺序相同

print(R_fixed @ z) # [0, -1, 0]
print(R_body @ z) # [1, 0, 0] 
print(np.allclose(R_fixed, R_body)) #False 两者不相同

# 2.分步 和 一次算，结果一致  用绕定坐标系 先绕x后绕y来演示
print(Ry90 @ (Rx90 @ z)) #这是分步算 先算括号里面的
print((Ry90 @ Rx90) @ z) #这是一次性算完 
print(np.allclose(Ry90 @ (Rx90 @ z) , (Ry90 @ Rx90) @ z)) # True 两者完全一致

# 3.验证 交换顺序 结果不同 
print(Rx90 @ Ry90 @ z)
print(Ry90 @ Rx90 @ z)
print(np.allclose(Rx90 @ Ry90 @ z , Ry90 @ Rx90 @ z)) #False

# 4.验证 绕着一根轴转 可以交换顺序
print(Rx(np.pi/3) @ Rx(np.pi/6) @ z)
print(Rx(np.pi/6) @ Rx(np.pi/3) @ z)
print(np.allclose(Rx(np.pi/6) @ Rx(np.pi/3) @ z , Rx(np.pi/3) @ Rx(np.pi/6) @ z)) # True

# 5.六种排列
for order in itertools.permutations([("x",Rx90),("y",Ry90),("z",Rz90)]): # itertools.permutations(...) 自动生成这 3 个元素的所有排列组合 3! = 6 种
# for order in ...：每次循环，order 会拿到一种排列 比如可能是 (("x", Rx90), ("y", Ry90), ("z", Rz90))
    R = np.eye(3)
    names = []
    for name, mat in order:  # 把order元组解包，name 拿到字符串（比如 "x"），mat 拿到具体的矩阵（比如 Rx90）
        R = mat @ R
        names.append(name)
        # 流程:例如：第一次mat拿到Rx90 R变成Rx90 name是["x"] 第二次 mat拿到Ry90 R变成Ry90 @ Rx90 name变成["x","y"]  第二次 mat拿到Rz90 R变成 Rz90 @ Ry90 @ Rx90 name变成["x","y","z]
    print("->".join(names), ":R @ z = ", np.round(R @ z, 3)) # np.round(..., 3)：四舍五入保留 3 位小数
    # "->".join(names)：把列表 ["x", "y", "z"] 拼接成字符串 "x->y->z"