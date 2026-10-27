# 用齐次变换重写FK （两连杆+平面 实际上就是简化版D-H建模）（牢记齐次变换的含义推导Pi = Tij × Pj）

# 1.为什么要用两种方法写同一件事情
# 昨天用的是几何推导 准确说叫三角几何建模法
# x = L1·cosθ1 + L2·cos(θ1 + θ2)
# y = L1·sinθ1 + L2·sin(θ1 + θ2)
# 它只在「两连杆 + 平面」这种情况下好用。 换成 6 轴机械臂、3D 空间，你根本画不出那个图。

# 齐次变换是通用的： 每一节就是一个 4×4 矩阵，串起来乘一下就完事 —— 不管几轴、几维。
# 今天要做的事：用齐次变换算出和昨天一模一样的答案 对上了即两套思路都对：用两条独立路径算同一个结果

# 2.每一节 = 一个4×4的矩阵
# 一个连杆的动作:先绕 z 转 θ， 再沿着它自己的新的方向伸出L
# def link_transform(theta, L):
#    R = Rz(theta)
#    T = np.eye(4)
#    T[:3, :3] = R
#    T[:3,  3] = R @ np.array[L, 0.0, 0.0] 
#    为什么是 R @ [L,0,0]，不是 [L,0,0] 因为连杆是沿着它自己转过去之后的方向伸出去的 ，不是沿着世界坐标系的X轴
#    T = [[R,t],[0,1]] 里的 t 是在父坐标系里表达的。你要让连杆沿自身方向伸出 L，就得先把 [L,0,0] 转到父坐标系 —— 也就是 R @ [L,0,0]  即想让它在自身坐标系里，就得先乘 R
 
# 3.串联：一路右乘 因为每一节的坐标系都是在上一节的坐标系里面表达的 即T（下一节变换）是基于上一节末端的坐标系来做的，也就是下一节的自身坐标系，即动坐标系
# T = link_transform(theta1, L1) @ link_transform(theta2, L2)
# end = T[:3, 3]   提取平移向量

# 再讲简单一点：右乘（动坐标系）：T1 @ T2。小臂绕着自己（大臂末端）转。（串联机械臂的标配）
#              左乘（定坐标系）：T2 @ T1。小臂不管大臂怎么动，非要固执地绕着世界绝对原点（或者底座）转。这在单条机械臂的运动学里是错的

# 4.代码训练
import numpy as np

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s,0],
                     [s, c,0],
                     [0, 0,1]])

# 昨天的解法：
def fk_2link(theta1, theta2, L1=1.0, L2=1.0):
    x = L1*np.cos(theta1) + L2*np.cos(theta1 + theta2)
    y = L1*np.sin(theta1) + L2*np.sin(theta1 + theta2)

    return np.array([x, y])

# 今天的齐次变换：
def link_transform(theta, L):
    # 一个连杆 绕z轴转theta，再沿自身x伸出L
    R = Rz(theta)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = R @ np.array([L, 0.0, 0.0]) # 必须先把(L，0 ，0)这个坐标转到动坐标系里面
    return T

def fk_2link_homo(theta1, theta2, L1 = 1.0, L2 = 1.0):  # 2连杆机械臂的正运动学（返回齐次变换矩阵）
    T = link_transform(theta1, L1) @ link_transform(theta2, L2)
    return T[:3, 3][:2]  # 提取平移 提取二维坐标 python中可以这样写

# (1) 对账：八组角度，两种方法必须一致
print("======对账======")
cases = [                                    # case 用例 其实就是python列表list 里面装了一堆元组tuple（不可变的列表）
    (0, 0), (np.pi/2, 0), (0, np.pi/2), (np.pi, 0), (0, np.pi),
    (0.3, 0.7), (-1.2, 2.1), (np.pi/3, -np.pi/4),
]

all_OK = True

for t1, t2 in cases:
    a = fk_2link(t1, t2)
    b = fk_2link_homo(t1, t2)
    OK = np.allclose(a, b)
    all_OK &= OK # 只要 cases 里面有任何一次计算 OK 变成了 False，all_OK 就永远变成 False 了，一票否决制
    print(f"θ1={t1:6.2f} θ2={t2:6.2f}  几何={np.round(a,4)}  齐次={np.round(b,4)}  {'✓' if OK else '✗'}")

print("\n全部一致：", all_OK)

# (2) 看中间矩阵长啥样（理解用）
print(np.round(link_transform(np.pi/2, 1.0), 3))

# (3) 反例：平移写成世界坐标会怎么样
def link_transform_wrong(theta, L):
    R = Rz(theta)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = np.array([L, 0.0, 0.0])     # ← 世界坐标（错）
    return T

def fk_wrong(t1, t2, L1=1.0, L2=1.0):
    T = link_transform_wrong(t1, L1) @ link_transform_wrong(t2, L2)
    return T[:3, 3][:2]

print("\n=== 反例对比 θ1=90°, θ2=0 ===")
print("正确:", np.round(fk_2link_homo(np.pi/2, 0), 3))   # [0, 2]
print("错误:", np.round(fk_wrong(np.pi/2, 0), 3))        # [1, 1]  ← 差很远  一个左右颠倒的L