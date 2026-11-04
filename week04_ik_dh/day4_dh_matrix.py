# 今天是重头戏 DH建模法 详细推导与图像展示见机器人学书
# 1.D-H四个参数分别是什么
# 参数             名字                  怎么动                       物理含义
#  θ             关节角                 绕Z轴转动                 两根连杆之间的夹角
#  d            连杆偏距                沿Z轴移动              两根连杆沿转轴方向的偏移
#  a            连杆长度                沿X轴移动             两关节轴线之间的公垂线长度
#  α            连杆扭转                绕X轴转动              两关节轴线之间的夹角 下一个转轴相对当前转轴「扭」了多少

# 转换矩阵 四个参数按顺序累乘
# T_i = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
#       └──────┬──────────┘ └───┬────┘
#       先在【z】上动手      再在【x】上动手
# ============================================================

# 想象从「坐标系 i-1」走到「坐标系 i」
# ① Rz(θ)   绕自己的 z 轴转动 θ     → 把 x 轴转到「公垂线方向」
# ② Tz(d)   沿自己的 z 轴移动 d     → 走到公垂线的起点
# ③ Tx(a)   沿自己的 x 轴移动 a     → 沿公垂线走完这一段
# ④ Rx(α)   绕自己的 x 轴转动 α     → 把 z 轴扳到下一个关节的方向
# 每一步都是在当前前进方向上的下一步 所以是一路右乘

# 注意：本文件用【标准型 D-H】：
#   T_i = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
# 参数下标都是 i。
#
# 不是 Craig 改进型（Rx(α_{i-1})·Tx(a_{i-1})·Rz(θ)·Tz(d)）
# 两套不能混用 —— 混用差了近 1 米，程序不报错。
# ============================================================

# 2.四个矩阵
# def Rz(t):              绕z轴旋转
#    c, s = np.cos(t), np.sin(t)
#    return np.array([
#        [c, -s, 0, 0],
#        [s,  c, 0, 0],
#        [0,  0, 1, 0],
#        [0,  0, 0, 1]])

# def Tz(d):             沿 Z 轴平移
#    T = np.eye(4)
#    T[2, 3] = d     代表第3行 第4列的元素 即z轴平移量
#    return T

# def Tx(a):             沿 X 轴平移
#   T = np.eye(4)
#   T[0, 3] = a      代表第1行 第4列的元素 即x轴平移量
#   return T

# def Rx(al):            绕 X 轴旋转
#    c, s = np.cos(al), np.sin(al)
#    return np.array([
#        [1, 0,  0, 0],
#        [0, c, -s, 0],
#        [0, s,  c, 0],
#        [0, 0,  0, 1]])

# 课本闭式矩阵：
#       ⎡ cθ   -sθ·cα    sθ·sα    a·cθ ⎤
# T_i = ⎢ sθ    cθ·cα   -cθ·sα    a·sθ ⎥
#       ⎢ 0     sα       cα        d   ⎥
#       ⎣ 0     0        0         1   ⎦

# 代码训练：
# W4 Day 4：完整 D-H 四参数
# T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
import numpy as np
np.set_printoptions(precision=6, suppress=True)  # 控制精度 打印浮点数时 保留小数点后6位 suppress=True 压制科学计数法  类似 6.123234e-17 直接打印为0

# 四个初等矩阵 自己写一遍
def Rz(t):    # 先初始化一个单位阵/零阵，然后再局部修改 是工业界的标准规范
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
def dh_transform(theta, d, a, alpha):
    return Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)

# 闭式矩阵
def dh_closed(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,      1],
    ])

# 三个验证
# (1)拆开累乘vs闭式 必须精确相等
rng = np.random.default_rng(0)  # 创建一个专门生成随机数的机器，并给它起名叫 rng
worst = 0.0   # 记录最大误差 目前是0
for _ in range(2000):   # 把下面的测试动作重复2000次
    th, d, a, al = (rng.uniform(-np.pi, np.pi), rng.uniform(-1, 1),
                    rng.uniform(-1, 1), rng.uniform(-np.pi, np.pi))
    err = np.max(np.abs(dh_transform(th, d, a, al) - dh_closed(th, d, a, al)))
    worst = max(worst, err)  # 更新最大误差
print(f"拆开乘 vs 闭式:  最大偏差 = {worst:.3e}")  # .3e表示保留3位小数

# (2)验证令d=0, α=0 → 必须等于 W3 的 link_transform
def link_transform_w3(theta, L):
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta),  np.cos(theta), 0],
                  [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = R @ np.array([L, 0.0, 0.0])
    return T

worst = 0.0
for _ in range(2000):
    th = rng.uniform(-np.pi, np.pi)
    L  = rng.uniform(0.1, 2.0)
    err = np.max(np.abs(dh_transform(th, 0.0, L, 0.0) - link_transform_w3(th, L)))
    worst = max(worst, err)
print(f"D-H(d=0,α=0) vs W3 link_transform:  最大偏差 = {worst:.3e}")

# (3)一个手算例子
T = dh_transform(np.pi/3, 0.2, 0.5, np.pi/2)
print(T)

# 手动核算
# 平移列 (0.25, 0.433013, 0.2) —— 正好是 (a·cosθ, a·sinθ, d) = (0.5×0.5, 0.5×0.866, 0.2)
# T[2,1] = 1 —— 这是 sinα，α=π/2 当然是 1
# T[0,2] = 0.866025 —— 这是 sθ·sα = sin(π/3)×sin(π/2) = 0.866 ✓