# 任务A 上述公式告诉你力或力矩要多大 但在写代码模拟物理世界时，电脑需要知道的是加速度θ̈ 
# 所以进行公式移项 变成 θ̈ =(τ−mglsinθ)/(ml2) 

# 任务B 测周期 把单摆放到不同初始角度（比如0.05 弧度、1.5 弧度），松手让它自由摆动（不给任何力矩）
# 写循环程序，记录它两次越过最低点的时间，算出“实测周期”  与理论周期T0对比，如下：
# T0 = 2π√(l/g) = 2.006067 s
# 当初始角是 0.05 时，测出的周期是 2.0064，几乎完美吻合

# 任务C 对比线性与非线性 
# 大学物理书里为了好算，经常把 sinθ 强行近似成 θ（小角度近似）。代码里写个开关，一个跑真实的 sinθ，另一个跑近似的 θ
# 非线性：θ̈ = -(g/l)·sinθ
# 线性化：θ̈ = -(g/l)·θ       ← 把 sinθ 近似成 θ

# 看实测数据 当初始角到了 1.5 弧度（接近 90 度）时，如果用线性模型去跑，误差会高达 16.2%！
# 这告诉你一个残酷的工程现实：如果机器人要抡大臂，千万不要用线性化模型，会翻车的

# 任务D 重力补偿
# 核心代码：tau_ff = m * g * l * np.sin(theta) 它代表：在任意角度 theta 下，重力究竟有多大 在任意情况下，都把重力给补偿掉了

# 现象 1：魔法悬停（纯前馈）
# 操作：不给 PID，只给 tau_ff。把单摆放到任意角度（比如 1.0 弧度），松手
# 现象：单摆完全静止，像冻在空气中一样。
# 物理真相：你实时算出了重力（比如往下的 9.8 牛），并且给电机输入了刚好往上的 9.8 牛。受力平衡，加速度为 0，单摆不动。 这证明你的动力学公式完全正确。

# 现象 2：回到 0 度（前馈 + PID）—— 为什么 I 项失业？
# 操作：在刚才前馈（抵消重力）的基础上，加上 PID，目标是让单摆回到 0 度
# 物理真相：因为前馈已经把重力抵消了，单摆现在处于“失重”状态（就像在太空中）
# PID 视角：
# P 项轻轻推一把：“往 0 度走”
# 因为没重力阻碍，单摆嗖地就过去了
# I 项（积分）根本没有机会累积。I 项累积的前提是“误差持续存在”。现在误差瞬间就被 P 项解决了，I 项一直饿着肚子，几乎保持在 0



import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 全局物理参数
# ==========================================
g = 9.81
l = 1.0
m = 1.0
dt = 1e-4  # 时间步长，越小越精确（教程要求 1e-4）

# ==========================================
# 任务 A：动力学方程（反解角加速度）
# ==========================================
def pendulum_dynamics(theta, omega, tau):
    """
    返回角加速度 ddtheta
    由 tau = m*l^2*ddtheta + m*g*l*sin(theta) 反解得到
    """
    ddtheta = (tau - m * g * l * np.sin(theta)) / (m * l**2)
    return ddtheta

# ==========================================
# 任务 B & C：测周期（非线性 vs 线性）
# ==========================================
def measure_period(theta0, is_linear=False, T_max=30.0):
    """
    从初始角 theta0 释放，测量过零点间隔 * 2 得到周期
    is_linear: True 则使用 sin(theta) ≈ theta 的线性模型
    """
    theta = theta0
    omega = 0.0
    t = 0.0
    prev_theta = theta
    
    t_cross1 = None
    t_cross2 = None
    
    while t < T_max:
        # 计算加速度
        if is_linear:
            ddtheta = -(g/l) * theta
        else:
            ddtheta = pendulum_dynamics(theta, omega, tau=0.0) # 自由摆动，tau=0
            
        # 欧拉积分
        omega += ddtheta * dt
        theta += omega * dt
        t += dt
        
        # 1. 找第一个过零点（从正数变负数，即从右侧荡回中间）
        if t_cross1 is None and prev_theta > 0 and theta <= 0:
            # 线性插值求出精确的过零点时间，避免离散误差
            t_cross1 = t - dt + (prev_theta / (prev_theta - theta)) * dt
            
        # 2. 找第二个过零点（从负数变正数，即荡到左侧）
        elif t_cross1 is not None and t_cross2 is None and prev_theta < 0 and theta >= 0:
            t_cross2 = t - dt + (prev_theta / (prev_theta - theta)) * dt
            break # 两个零点都找到了，直接结束
            
        prev_theta = theta
        
    if t_cross1 is not None and t_cross2 is not None:
        # 两个相邻过零点之间的时间差是半个周期
        return (t_cross2 - t_cross1) * 2
    return None

# ==========================================
# 任务 D 的准备：PID 类
# ==========================================
class PID:
    def __init__(self, kp, ki, kd, u_lim=50.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.u_lim = u_lim
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_val = None

    def update(self, target, current, dt):
        e = target - current
        if self.prev_val is None:
            de = 0.0
        else:
            de = -(current - self.prev_val) / dt
        self.prev_val = current

        raw = self.kp * e + self.ki * self.integral + self.kd * de
        u = max(-self.u_lim, min(self.u_lim, raw))
        
        # 条件积分抗饱和
        if u == raw:
            self.integral += e * dt
        return u

# ==========================================
# 任务 D：重力补偿（前馈） + PID
# ==========================================
def simulate_gravity_comp(theta0, target=0.0, use_pid=False, use_ff=True, T_max=5.0):
    """
    theta0 : 初始角度
    target : 想让它【保持】的角度   ← 关键！不能总是 0
    use_ff : 是否加前馈（重力补偿）
    """
    pid = PID(kp=80, ki=30, kd=3) if use_pid else None
    theta = theta0
    omega = 0.0
    t = 0.0

    t_hist, theta_hist, integral_hist = [], [], []

    while t < T_max:
        t_hist.append(t)
        theta_hist.append(theta)
        if use_pid:
            integral_hist.append(pid.integral)

        # 核心 1：前馈（提前算出重力需要多少力）—— 可以关掉做对照
        tau_ff = m * g * l * np.sin(theta) if use_ff else 0.0

        # 核心 2：反馈（PID 修正微小误差）—— 目标是 target，不是写死的 0
        tau_fb = pid.update(target, theta, dt) if use_pid else 0.0

        # 总力矩
        tau = tau_ff + tau_fb
        
        # 物理引擎步进
        ddtheta = pendulum_dynamics(theta, omega, tau)
        omega += ddtheta * dt
        theta += omega * dt
        t += dt
        
    if use_pid:
        return np.array(t_hist), np.array(theta_hist), np.array(integral_hist)
    else:
        return np.array(t_hist), np.array(theta_hist)

# ==========================================
# 主程序：跑测试并打印中文表格
# ==========================================
if __name__ == "__main__":
    print("========== 任务 A & B: 非线性周期测量 ==========")
    T0_theory = 2 * np.pi * np.sqrt(l/g)
    print(f"理论周期 T0 = {T0_theory:.6f} s\n")
    
    angles = [0.05, 0.10, 0.30, 0.50, 1.00, 1.50]
    print(f"{'初始角':>6} | {'实测周期':>10} | {'相对T0':>8} | {'偏差':>8}")
    print("-" * 45)
    
    for theta0 in angles:
        T_meas = measure_period(theta0, is_linear=False)
        relative_T0 = T_meas / T0_theory
        error = (T_meas - T0_theory) / T0_theory * 100
        print(f"{theta0:>6.2f} | {T_meas:>10.4f} | {relative_T0:>8.4f} | {error:>+7.2f}%")
        
    print("\n========== 任务 C: 线性 vs 非线性 ==========")
    print(f"{'初始角':>6} | {'非线性实测':>10} | {'线性实测':>10} | {'线性误差':>10}")
    print("-" * 45)
    for theta0 in [0.1, 0.5, 1.0, 1.5]:
        T_nonlin = measure_period(theta0, is_linear=False)
        T_lin = measure_period(theta0, is_linear=True)
        # 线性的解析周期永远是 T0
        error = (T_lin - T_nonlin) / T_nonlin * 100
        print(f"{theta0:>6.2f} | {T_nonlin:>10.4f} | {T_lin:>10.4f} | {error:>+9.2f}%")

    print("\n========== 任务 D: 重力补偿 ==========")
    # 实验 1：纯重力补偿 (定住不动)
    _, theta_D1 = simulate_gravity_comp(theta0=1.0, use_pid=False, T_max=2.0)
    print(f"D1 (纯前馈): 初始角 1.0 rad，最大波动: {np.max(np.abs(theta_D1 - 1.0)):.2e} (极接近 0)")

    # ---------------------------------------------------------------
    # 实验 2（修正版）：让单摆【保持】在 θ = 0.5 rad，对比有无前馈
    #
    #   ⚠️ 原来这里目标是 0 —— 但 θ=0 处重力矩 m·g·l·sin(0) = 0，
    #      前馈在那里【无事可做】，所以看不出任何差别。
    #      必须选一个重力矩不为零的角度，前馈的价值才显现。
    # ---------------------------------------------------------------
    TARGET = 0.5
    NEED = m * g * l * np.sin(TARGET)
    print(f"\nD2 (保持 {TARGET} rad): 维持这个姿态需要 τ = m·g·l·sin({TARGET}) = {NEED:.4f} N·m")
    print(f"{'':<22}{'终态角度':>12}{'稳态积分':>12}{'积分峰值':>12}")

    # A) 纯 PID，没有前馈 —— 重力全靠积分器扛
    t_A, th_A, int_A = simulate_gravity_comp(
        theta0=0.0, target=TARGET, use_pid=True, use_ff=False, T_max=5.0)
    ss_A = np.mean(np.abs(int_A[-int(1.0/dt):]))
    print(f"{'A) 纯 PID（无前馈）':<22}{th_A[-1]:>12.6f}{ss_A:>12.5f}{np.max(np.abs(int_A)):>12.5f}")

    # B) 前馈 + PID —— 重力提前算好，积分器只管补零头
    t_D2, th_B, int_B = simulate_gravity_comp(
        theta0=0.0, target=TARGET, use_pid=True, use_ff=True, T_max=5.0)
    ss_B = np.mean(np.abs(int_B[-int(1.0/dt):]))
    print(f"{'B) 前馈 + PID':<22}{th_B[-1]:>12.6f}{ss_B:>12.5f}{np.max(np.abs(int_B)):>12.5f}")

    print(f"\n  → 稳态积分：A = {ss_A:.5f}   B = {ss_B:.5f}   "
          f"**差 {ss_A/max(ss_B,1e-12):.0f} 倍**")
    print(f"  → 理论：A 应该 ≈ τ/Ki = {NEED:.4f}/{30} = {NEED/30:.5f}"
          f"（实测略小，因为 P 项也出了一部分力）")
    print("  → 这就是「前馈让积分器失业」的证据")

    # 后面的画图仍用 B 的数据
    theta_D2, integral_D2 = th_B, int_B

    # ==========================================
    # 可视化：画图展示任务 D 的前馈威力（保持英文防乱码）
    # ==========================================
    plt.figure(figsize=(10, 4))
    
    # 左图：角度变化
    plt.subplot(1, 2, 1)
    plt.plot(t_D2, theta_D2, label="Theta (Feedforward + PID)", color='b')
    plt.axhline(0, color='k', linestyle='--')
    plt.title("Task D: Gravity Compensation + PID, hold at 0.5 rad")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (rad)")
    plt.legend()
    plt.grid(True)
    
    # 右图：积分项累加情况
    plt.subplot(1, 2, 2)
    plt.plot(t_D2, integral_D2, color='orange', label="PID Integral")
    plt.title("Integral Term: FF+PID stays near 0 (pure PID would hit 0.126)")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

# 必须自己会写、且闭卷能默写的（核心考点）


# 1.动力学公式（反解角加速度）：
# 必须能把 τ=ml2θ̈ +mglsinθ 移项成代码里的：
# ddtheta = (tau - m*g*l*np.sin(theta)) / (m*l**2)
# （面试必考：给你一个力矩，让你反求当前加速度）

# 2.重力补偿（前馈项）：
# 必须能手写出这句话：
# tau_ff = m * g * l * np.sin(theta)
#（面试必考：怎么让机械臂悬停在半空不耗电？答：用前馈把重力抵消掉）

# 3.前馈 + 反馈的组合逻辑：
# 得知道真实的控制代码长这样：
# u = tau_ff + pid.update(target, current, dt)
# （前馈提前算好重力大账，PID 负责修补剩下的微小误差）

# 4.W5 的 PID 类：
# 30 行的 PID 类，作为机器人工程师，闭着眼睛也得能敲出来（尤其是微分先行和抗饱和）

# 今天最重要的一句：
# τ = m·l²·θ̈ + m·g·l·sinθ —— 前一项是「要动就得给」，后一项是「不动也得给」
# W5 的 PID 在保持姿态时，积分器攒的正是第二项

# 什么是稳态积分：
# 就是系统进入稳态时，积分项（I）的值 纯pid时 稳态积分必须一直存在 用以抵抗重力 pid+前馈时 稳态积分是纯pid的42分之一 证明积分项几乎失效 因为重力已经被前馈抵消掉了