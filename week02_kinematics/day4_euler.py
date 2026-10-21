# 1.欧拉角是什么 （详见机器人学）
# 用三个角度描述一个3D旋转  
# 旋转矩阵有9个数值 人一眼能看懂的是3个自由度 欧拉角就是用3个数 "绕 z 转 30°" "绕 y 转 40°" "绕 x 转 20°" 轻松直观描述旋转过程
# 代价是必须约定：绕哪几根轴、按什么顺序

# 2.12种约定
# 三根轴排个序：XYZ、XZY、YXZ、YZX、ZXY、ZYX —— 6 种
# 每种再分定轴 / 动轴 —— ×2 = 12 种
# 还有"第一根和第三根相同"的经典欧拉角，如 ZXZ、ZYZ 
# 所以看到欧拉角 先问哪套约定  同一组数字 (30, 20, 10)，在 ZYX 和 XYZ 下代表完全不同的姿态
# 机器人和航天最常用的是ZYX （动坐标系），也就是yaw-pitch-roll（偏航 - 俯仰 - 横滚）：
# R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

# 3.万向锁（核心）
# 详细推导以及解释过程见机器人学p70
# ZYX 动坐标系：
# R = Rz(yaw) @ Ry(pitch) @ Rx(roll)  
# 当 pitch = 90° 时，Ry(90°) 把 x 轴送到 -z、把 z 轴送到 x。代进去化简之后，整个矩阵只依赖 roll - yaw 这一个量：
# R(pitch=90°) 只由 (roll − yaw) 决定 即yaw和roll不再各自独立决定姿态 只有它们的差有意义
# 简单来说 由于y轴转了90° 导致x和z两根轴共线了
# 代码验证见下文：用两组不同的yaw roll 但roll-yaw相同
# 两个不同的输入，同一个输出。 意味着从姿态反推不出 yaw 和 roll 分别是多少 —— 信息丢了
# 无法唯一反解 以为有3个自由度 其实只有2个

# 4.代码训练
import numpy as np

# 三个绕轴转的基本定义
def Rx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0,  0], 
                     [0, c, -s], 
                     [0, s,  c]])

def Ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c, 0, s], 
                     [ 0, 1, 0], 
                     [-s, 0, c]])

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], 
                     [s,  c, 0], 
                     [0,  0, 1]])

# 定义ZYX函数
def R_from_ypr(yaw, pitch, roll): # 这里输入三个弧度
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)

z = np.array([0.0, 0.0, 1.0])
d = np.deg2rad #np.deg2rad是角度转弧度函数 比如np.deg2rad(90) 就是把90°转为pi/2 这里是把这个函数取名为d

# （1）正常情况 不同角度 -> 不同姿态
A = R_from_ypr(d(30), d(20), d(10))
B = R_from_ypr(d(10), d(20), d(30))
print(A @ z)
print(B @ z)
print(np.allclose(A @ z , B @ z)) # False

# （2）万向锁：pitch = 90°  (30,90,10) (40,90,20)  yaw - roll = 20°
C = R_from_ypr(d(30), d(90), d(10))
D = R_from_ypr(d(40), d(90), d(20))
print(C @ z)
print(D @ z)
print(np.allclose(C @ z , D @ z)) # True 两个不同旋转矩阵输出的结果是一样的 这就导致从姿态出发没法求解

# （3）验证当 pitch=90°时候 只有改变 yaw - roll 才会改变最终的姿态
E = R_from_ypr(d(30), d(90), d(10))
F = R_from_ypr(d(50), d(90), d(15))
print(np.round(E @ z , 3)) # np.round保留三位小数
print(np.round(F @ z , 3))
print(np.allclose(E @ z , F @ z)) # False