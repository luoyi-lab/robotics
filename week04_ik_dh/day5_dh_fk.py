# 从dh_table生成FK 和独立模型对账
import numpy as np
L1, L2 = 1.0, 0.8

def Rz(t):   
    c, s = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s
    T[1, 0] = s; T[1, 1] =  c
    return T

def Tz(d):
    T = np.eye(4)
    T[2, 3] = d
    return T

def Tx(a):
    T = np.eye(4)
    T[0, 3] = a
    return T

def Rx(al):
    c, s = np.cos(al), np.sin(al)
    T = np.eye(4)
    T[1, 1] = c; T[1, 2] = -s
    T[2, 1] = s; T[2, 2] =  c
    return T

# 组合
def dh(theta, d, a, alpha):
    return Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)

# D-H参数表（例子B）
def fk_from_table(t1, t2):
    T1 = dh(t1, 0, L1, np.pi/2)
    T2 = dh(t2, 0, L2, 0.0)
    T = T1 @ T2
    return T[:3, 3]
# 左上角的 3×3矩阵（R）：代表末端执行器在三维空间中的姿态（旋转）。
# 右上角的 3×1向量（x,y,z）：代表末端执行器在基座坐标系下的位置（平移）。

# 独立几何模型 不经过D-H
def fk_geom(t1, t2):
    # 肩转 t1，肘在竖直平面里仰 t2。这个不看 D-H，纯几何
    d_hat = np.array([np.cos(t1), np.sin(t1), 0.0])  # 大臂方向向量
    z = np.array([0.0, 0.0, 1.0])  # Z轴单位向量
    return L1 * d_hat + L2 * (np.cos(t2) * d_hat + np.sin(t2) * z)  # 将小臂向量分解：水平分量是 L2 * cos(t2)，方向和大臂一样（d_hat）；竖直分量是 L2 * sin(t2)，方向竖直向上（z）

# 对账
# 1. 跑 2000 组随机角度
np.random.seed(42)
max_error = 0.0

for _ in range(2000):
    t1_rand = np.random.uniform(-np.pi, np.pi)
    t2_rand = np.random.uniform(-np.pi, np.pi)
    
    # 方法 A: 矩阵连乘 (D-H)
    pos_dh = fk_from_table(t1_rand, t2_rand)
    # 方法 B: 向量几何
    pos_geom = fk_geom(t1_rand, t2_rand)
    
    # 计算两者之间的距离误差
    error = np.linalg.norm(pos_dh - pos_geom)
    if error > max_error:
        max_error = error

print(f"最大偏差: {max_error:.3e}")