# 今天是维度上的升级 从 Day 1 的“力学推导（二阶微分方程）”跨越到了现代控制理论的基石：状态空间方程与离散化
# 为了以后学习 LQR（线性二次型调节器）、MPC（模型预测控制）铺平道路。今天的内容涉及矩阵和特征值

# 第一部分 状态空间 把二阶方程降维
# 状态空间就是：把系统的所有状态变量（如角度、速度），当成坐标轴撑起来的一个多维数学空间
# 为什么需要：
# day1的单摆方程是二阶（θ̈ ）的：m·l²·θ̈ = τ − m·g·l·sinθ
# 二阶方程不好用矩阵工具分析 所以定义一个状态向量x将它拆成一阶
# x = [ θ  ]        ← 位置
#     [ θ̇  ]        ← 速度
# 两条一阶方程：
# ẋ₁ = θ̇  = x₂
# ẋ₂ = θ̈  = −(g/l)·sinθ + τ/(m·l²)
# 线性化 sinθ ≈ θ 之后：
# ẋ₁ = x₂
# ẋ₂ = −(g/l)·x₁ + (1/(m·l²))·τ
# 要写成矩阵公式，即如下形式：（推导见每日记录本子）
# ẋ = A·x + B·u
# 
# A = ⎡ 0      1  ⎤        B = ⎡    0      ⎤
#     ⎣ −g/l   0  ⎦            ⎣ 1/(m·l²)  ⎦
# 
# 输出方程：y = C·x + D·u        C = [1  0]      D = [0]
#                ↑ 你能测到的量（这里只能测角度）
# 在这个单摆系统里，你的系统有两个隐藏状态：x1=θ（角度）和 x2=θ˙（角速度）。
# 但是，现实中的传感器是有局限的。假设你只装了一个电位计/编码器，它只能测量角度，测不到速度。所以，你能看到的输出 
# y，仅仅就是 x1  即y=θ
# C矩阵是选择矩阵 D矩阵表示直接传递项

# 第二部分 看A的特征值  代码是：np.linalg.eigvals(A)   →   [0.+3.132j, 0.-3.132j]
# 特征值的定义是：解方程 det(A−λI)=0  解得λ = ±jω 结果如下：

# 特征值	含义
# 实部 = 0	无阻尼（能量不散失），所以会一直振荡
# 虚部 = ±3.132	振荡角频率 ω
# 2π/ω = 2.006067 s	← 和昨天实测的周期完全一致

# 神迹发生：你在 Day 1 跑仿真测出来的周期是 2.006067 秒，而这里你连仿真的代码都没写，
# 仅仅用一个矩阵的特征值，就算出了完全一样的答案！这就是数学的威力：不用仿真，看矩阵就知道系统怎么动

# 第三部分 连续变离散 
# 真实世界是连续的，但控制器是一台计算机，它是离散的，每 dt秒（比如 0.01 秒）才算一次 所以我们的模型也必须是离散的：
# ẋ = Ax + Bu    →     x[k+1] = Ad·x[k] + Bd·u[k]    k代表第k个控制周期  x[k] 是当前这一帧的状态  x[k+1] 是下一帧的状态
# 怎么才能连续矩阵 A B变成离散矩阵 Ad Bd？有两种做法：
# 做法一：前向欧拉：
# Ad = I + A*dt
# Bd = B*dt
# 也就是假设这 dt 时间内，导数不变。但这是个致命陷阱

# 做法二：零阶保持器（ZOH，精确无敌）
# Ad = expm(A*dt)                    # 矩阵指数
# Bd = ∫₀^dt expm(A*τ)dτ · B

# 它使用了指数矩阵 expm(A*dt) 对单摆这种特殊矩阵，它有完美的解析解如下：
# c, s = cos(ω*dt), sin(ω*dt)
# Ad = [[ c,      s/ω ],
#       [ -ω*s,   c   ]]

# 第四部分：核心实验
# 把初始状态设为 [0.1,0]（初始角度0.1，速度为0），分别用欧拉法和 ZOH 跑 10 秒，看最终振幅，即单摆偏离中心位置（最低点，也就是 0 度）的最大角度

import os
import numpy as np
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

# ==========================================
# 参数设置
# ==========================================
g, l, m = 9.81, 1.0, 1.0
omega = np.sqrt(g/l)
print(f"特征值计算: w = {omega:.3f}")
print(f"理论周期: {2*np.pi/omega:.6f} s\n")

# 连续矩阵 A (状态空间)
A = np.array([[0, 1], [-g/l, 0]])
print("A 矩阵:\n", A)

# 求特征值
eigenvals = np.linalg.eigvals(A)
print(f"A 的特征值: {eigenvals}\n")

# ==========================================
# 方法 A：前向欧拉法离散化
# ==========================================
def get_Ad_Euler(dt):
    return np.eye(2) + A * dt

# ==========================================
# 方法 B：ZOH 精确离散化 (单摆解析解)
# ==========================================
def get_Ad_ZOH(dt):
    c = np.cos(omega * dt)
    s = np.sin(omega * dt)
    return np.array([
        [c, s/omega],
        [-omega*s, c]
    ])

# ==========================================
# 核心实验：跑 10 秒，看振幅
# ==========================================
dt_list = [0.002, 0.010, 0.050, 0.100, 0.300, 0.500]
T_total = 10.0
x0 = np.array([0.1, 0.0]) # 初始角度 0.1，速度 0

print(f"{'dt':>6} | {'w*dt':>8} | {'欧拉 10s 后 |θ|':>15} | {'ZOH 10s 后 |θ|':>15}"
      f" | {'实际跑多久':>10} | {'ZOH 全程峰值':>12}")
print("-" * 92)

euler_results = []
zoh_results = []
zoh_peak = []

for dt in dt_list:
    # ⚠️ 用 round 而不是 int：int(10.0/0.3) = 33 → 实际只跑了 9.9 s，
    #    末端 |θ| 会因为相位差变成 0.0918，看着像 ZOH 也衰减了 —— 其实是假象
    steps = int(round(T_total / dt))
    T_actual = steps * dt

    # 跑欧拉
    x_euler = x0.copy()
    Ad_euler = get_Ad_Euler(dt)
    for _ in range(steps):
        x_euler = Ad_euler @ x_euler
    euler_amp = abs(x_euler[0])

    # 跑 ZOH
    x_zoh = x0.copy()
    Ad_zoh = get_Ad_ZOH(dt)
    peak = 0.0
    for _ in range(steps):
        x_zoh = Ad_zoh @ x_zoh
        peak = max(peak, abs(x_zoh[0]))
    zoh_amp = abs(x_zoh[0])

    euler_results.append(euler_amp)
    zoh_results.append(zoh_amp)
    zoh_peak.append(peak)

    print(f"{dt:>6.3f} | {omega*dt:>8.4f} | {euler_amp:>15.4f} | {zoh_amp:>15.4f}"
          f" | {T_actual:>10.4f} | {peak:>12.6f}")

print("\n→ ZOH 那一列的 0.0995 不是 0.1，是【相位】不是衰减：0.1*|cos(w*T)| = "
      f"{0.1*abs(np.cos(omega*T_total)):.6f}")
print("→ 全程峰值才是真正的「振幅」：ZOH 每一行都 = 0.1，一个数都不掉")

# ==========================================
# 补一刀：离散矩阵自己的特征值 = e^(±jωdt)，模长必须【恰好】等于 1
#   —— 这才是「ZOH 守恒」的数学证明，不用跑仿真
# ==========================================
print(f"\n{'dt':>6} | {'eig(Ad_ZOH)':>34} | {'模长':>10} | {'欧拉 |I+Adt|':>14}")
print("-" * 76)
for dt in dt_list:
    ev = np.linalg.eigvals(get_Ad_ZOH(dt))
    ev_e = np.linalg.eigvals(get_Ad_Euler(dt))
    print(f"{dt:>6.3f} | {str(np.round(ev, 6)):>34} | {str(np.round(np.abs(ev), 9)):>10} "
          f"| {str(np.round(np.abs(ev_e), 6)):>14}")
print("→ ZOH：模长恒为 1（等幅振荡）；欧拉：模长 = √(1+(ωdt)²) 恒 > 1（必发散）")

# ==========================================
# 画图：横轴 dt (对数)，纵轴 10秒后振幅
# ==========================================
plt.figure(figsize=(8, 5))
plt.plot(dt_list, euler_results, 'o-', color='red', label='Forward Euler (Explodes)')
plt.plot(dt_list, zoh_results, 's-', color='green', label='ZOH (Perfectly Preserved)')
plt.axhline(0.1, color='black', linestyle='--', label='True Amplitude (0.1)')

plt.xscale('log') # 横轴用对数坐标，看得更清楚
plt.yscale('log') # 纵轴也用对数，因为欧拉法爆炸到8703太夸张
plt.xlabel('dt (Time Step, log scale)')
plt.ylabel('Amplitude after 10s (log scale)')
plt.title('Forward Euler vs ZOH Discretization')
plt.legend()
plt.grid(True, which="both", ls="--")

# WSL 里 plt.show() 弹不出窗口（W5 踩过的坑）—— 存图才是产物
out_png = os.path.join(HERE, "day2_discretization.png")
plt.savefig(out_png, dpi=150, bbox_inches="tight")
print(f"\n图已保存: {out_png}")
# plt.show()








