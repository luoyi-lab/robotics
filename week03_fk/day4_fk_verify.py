import pybullet as p
import numpy as np
import os
import math
HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "day3_two_link_arm_2.urdf")
print("URDF 路径:", URDF)

# 注意 这两天的代码都是为了展示对比 调包 不需要特别记忆或者手搓


# 第一步 摸清 getLinkState 返回什么
p.connect(p.DIRECT)  # 链接物理引擎
arm = p.loadURDF(URDF, useFixedBase=True)  # 加载机器人模型

# 把两个关节设到指定角度
p.resetJointState(arm, 0,  0.5)  #joint1   0号关节设为 0.5 弧度
p.resetJointState(arm, 1, -0.8)  #joint2   1号关节设为-0.8 弧度

# 读 link2 的位姿（index = 1）
ls = p.getLinkState(arm, 1, computeForwardKinematics=True)
# 这里getLinkState返回6个元素
# [0]   世界系下的质心位置
# [1]   世界系下的质心姿态（四元数）
# [2][3]    局部坐标系下的质心位置与姿态
# [4][5]    世界系下的连杆原点位置与姿态


print(f"返回了 {len(ls)} 个元素：")
for i, v in enumerate(ls):
    print(f"  [{i}] {v}")

print()
print("=== 官方字段名 ===")
print(p.getLinkState.__doc__)

# 第二步 用正确的索引算出末端位置

def quat_to_matrix(q):
    #  注意：PyBullet 返回的是 (x, y, z, w)，而标准公式通常是 (w, x, y, z)
    # 所以如果你直接传 ls[5] 进来，要先把顺序调换一下
    x, y, z, w = q  
    return np.array([[1-2*(y*y+z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
                     [  2*(x*y + w*z), 1-2*(x*x+z*z),   2*(y*z - w*x)],
                     [  2*(x*z - w*y),   2*(y*z + w*x), 1-2*(x*x+y*y)],
                   ])

def pybullet_fk(t1, t2):
    # 让 PyBullet 设角度，返回末端的 (x, y)
    p.resetJointState(arm, 0, t1)
    p.resetJointState(arm, 1, t2)

    ls = p.getLinkState(arm, 1, computeForwardKinematics=True)

    pos = ls[4]  # Link 2 坐标系原点在世界系下的位置
    quat = ls[5]  #  Link 2 坐标系原点在世界系下的姿态（四元数）

    R = quat_to_matrix(quat)

    tip = pos + R @ np.array([1.0, 0, 0]) # 核心：把 Link 1 自身的局部坐标 [1,0,0] 旋转到世界坐标系下

    return tip[:2]

# 第三步 八组角度对账
cases = [(0,0), (math.pi/2,0), (0,math.pi/2), (math.pi,0),
         (0,math.pi), (0.3,0.7), (-1.2,2.1), (math.pi/3,-math.pi/4)]

def fk_2link(theta1, theta2, L1 = 1.0, L2 = 1.0):
    # 两连杆机械臂正运动学（平面）
    # theta1 : 连杆1 相对 x 轴的角度（弧度）
    # theta2 : 连杆2 相对【连杆1】的角度（弧度）  ← 相对角
    # 返回    : 末端 (x, y)
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return np.array([x, y])

all_ok = True
for t1, t2 in cases:
    ours   = fk_2link(t1, t2)          # 你 Day 1 写的
    theirs = pybullet_fk(t1, t2)
    ok = np.allclose(ours, theirs, atol=1e-6)     # ← 程序判断
    all_ok &= ok # 做与运算
    print(f"θ1={t1:6.2f} θ2={t2:6.2f}  我们={np.round(ours,4)}  仿真的={np.round(theirs,4)}")
print("\n全部一致:", all_ok)