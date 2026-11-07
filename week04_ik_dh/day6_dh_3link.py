# α ≠ 0 的三连杆：真正的检验
# 机械臂：肩部转 大臂俯仰 小臂俯仰（手画图）

# D-H 表（标准型）：

#   i	θ	 d	    a	   α
#   1	θ1	 0.5    0	 +90°
#   2	θ2	 0	   1.0	   0
#   3	θ3	 0	   0.7	   0

import numpy as np

def Rz(t):
    c, s = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s
    T[1, 0] = s; T[1, 1] =  c
    return T

def Tz(d):
    T = np.eye(4); T[2, 3] = d; return T

def Tx(a):
    T = np.eye(4); T[0, 3] = a; return T

def Rx(al):
    c, s = np.cos(al), np.sin(al)
    T = np.eye(4)
    T[1, 1] = c; T[1, 2] = -s
    T[2, 1] = s; T[2, 2] =  c
    return T

# 标准型 D-H 变换
def dh(theta, d, a, alpha):
    return Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)


# 三连杆正运动学

def fk_dh(t1, t2, t3):
    T = (dh(t1, 0.5, 0.0, np.pi/2) @
         dh(t2, 0.0, 1.0, 0.0) @
         dh(t3, 0.0, 0.7, 0.0))
    return T[:3, 3] # 提取 x, y, z

if __name__ == "__main__":
    # 测试点1：全零位 (验证是不是 (1.7, 0, 0.5))
    p0 = fk_dh(0, 0, 0)
    print(f"全0位末端: ({p0[0]:.2f}, {p0[1]:.2f}, {p0[2]:.2f})")

    # 测试点2：θ2 抬 90度 (大臂竖直向上)
    p2 = fk_dh(0, np.pi/2, 0)
    print(f"θ2=90°末端: ({p2[0]:.2f}, {p2[1]:.2f}, {p2[2]:.2f})")

    # 测试点3：手算挑战题，θ1=90°, θ2=0, θ3=0
    p3 = fk_dh(np.pi/2, 0, 0)
    print(f"θ1=90°末端: ({p3[0]:.2f}, {p3[1]:.2f}, {p3[2]:.2f})")