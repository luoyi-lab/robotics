# 前两天的PID 在数学上是完美的 但是在真机上就会遇到两个极其致命的问题：积分饱和和微分冲击 今天就是解决这两个问题
# 第一部分 积分饱和 为什么我的车会飞向太空
# 场景：目标位置本来是 1，系统稳定得很好。第 5 秒，目标突然变成 20
# 代码执行器（电机）有个物理上限：比如最多只能输出 10 的力（U_LIM = 10.0）
# 发生了什么？

# 目标变成 20，误差 e = 19。

# 假设你算出来的理论推力（raw）是 10*19 = 190。

# 但是电机最多只能给 10，所以最终推力 u = 10。
# 致命Bug：虽然你的输出被限制在了 10，但你的积分项 integral 还在按 19 * dt 疯狂累加！
# 后果：车花了很久才走到 20，在这漫长的过程中，积分器已经攒了一个极其恐怖的数值（比如 190）。
# 当车终于快到 20 时，哪怕误差变成 0，积分项依然在疯狂输出巨大的推力。于是车直接冲过头，飞到 116.9，然后反向暴跌到 -212，彻底失控。

# 一句话总结：执行器已经累瘫了（输出饱和），但积分器还在认为自己没出力，疯狂记账

# 怎么解决：三种抗饱和方法
# 方法 A：钳位（Clamp）—— 最简单
# 做法：给积分项本身设一个上下限（比如限制在 ±6）。不管外面怎么累加，积分项最大就 6
# 代码：self.integral = max(-self.ilim, min(self.ilim, self.integral))

# 方法 B：条件积分（Conditional Integration）—— 最常用
# 做法：判断一下，如果算出来的 raw 超过了最大值（说明已经饱和了），这一拍就干脆别做积分累加了
# raw是理论完美数值raw = self.kp * e + self.ki * self.integral + self.kd * de
# u是现实中机器能给的最大值 比如最大只能给10N的力 U_LIM = 10
# 代码：if not saturated: self.integral += e * dt

# 方法 C：反计算（Back-calculation）—— 工业界常见
# 做法：把（理论想要输出的值 - 实际被限制后的输出值）乘上一个系数，反加回积分器里，强行把积分项拽回来
# kb = 0.5  # 反馈系数（往回拉的力量有多大）
# self.integral += (e + kb * (u - raw)) * dt
# 在方法B的基础上升级 不仅要踩刹车，还要主动把之前记错的账往回扣，甚至倒贴

# 规律：你的电机限制得越死（比如只能出 10 的力），目标跳得越远（1 到 20），不加抗饱和的 PID 炸得就越惨

# 第二部分 微分先行 为什么电机会瞬间抽搐？
#  现象与病因（对应图 5）
# 在传统的微分项里：de = (e - prev_error) / dt，而 e = target - current。
# 如果在第 5 秒，目标突然从 1 变成 20：

# 那么 e 会瞬间从 0 突变到 19。
# 如果 dt=0.01，那么 de = 19 / 0.01 = 1900。
# 如果 Kd = 5，D 项的输出 = 1900 * 5 = 9500！

# 发生了什么？ 真实世界里，小车的速度不可能瞬间改变，它是因为目标变了，导致 e 突变。你的 D 项算出了一个 9500 的瞬时尖峰力，
# 这种力一瞬间打在电机上，电机瞬间过流、触发保护、甚至烧毁齿轮。

# 实测：传统对误差求微分，D 项峰值高达 9500；改用微分先行后，D 项峰值只有 144
# 怎么解决？（微分先行）即对测量值求微分
# 做法：不要对 误差 (e) 求导，改对 测量值 (current) 求导。

# 数学原理：因为 e = target - current，当目标不变时，de/dt = -d(current)/dt。

# 物理意义：目标突变是“虚拟”的指令，但小车的实际速度变化是“物理”的。微分项（D）的本意是阻尼/刹车。我们只应该对真实的物理速度进行刹车，而不是对指令突变去刹车。

# 反直觉的事实：如果你只看输出位置的曲线，你会发现改不改微分先行，最终位置的曲线看起来几乎一模一样。但如果你看 u（控制信号）曲线，差别是生死存亡的（9500 vs 144）。所以一定要画出 u 随时间变化的曲线
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 最终完美版 PID 类
# ==========================================
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, u_lim=10.0, mode='none', derivative_on_measurement=False):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.u_lim = u_lim
        self.mode = mode
        self.derivative_on_measurement = derivative_on_measurement
        self.reset()

    def reset(self):
        """清空历史记忆，防止任务切换时残留数据干扰"""
        self.integral = 0.0
        self.prev_val = None
        self.prev_error = 0.0

    def update(self, target, current, dt):
        e = target - current
        
        # --- 核心 1：微分项（精简版） ---
        if self.prev_val is None:
            # 第一拍：预热，不产生任何冲击
            de = 0.0
        elif self.derivative_on_measurement:
            # 微分先行：只看测量值（物理速度）的变化，不管目标怎么突变
            de = -(current - self.prev_val) / dt
        else:
            # 传统微分：对误差求导，目标突变会引发尖峰
            de = (e - self.prev_error) / dt
            
        self.prev_error = e
        self.prev_val = current

        # --- 核心 2：计算理论输出与实际输出 ---
        raw = self.kp * e + self.ki * self.integral + self.kd * de
        u = max(-self.u_lim, min(self.u_lim, raw))

        # --- 核心 3：抗饱和策略 ---
        if self.mode == 'none':
            self.integral += e * dt
        elif self.mode == 'clamp':
            ilim = self.u_lim / self.ki if self.ki > 0 else 0
            self.integral += e * dt
            self.integral = max(-ilim, min(ilim, self.integral))
        elif self.mode == 'conditional':
            # 只有在没有饱和（输出没撞天花板）时，才累加积分
            if u == raw:
                self.integral += e * dt
        elif self.mode == 'backcalc':
            kb = 0.5
            self.integral += (e + kb * (u - raw)) * dt
            
        # 返回控制量和 D 项（用于打图证明）
        return u, self.kd * de


# ==========================================
# 仿真测试引擎
# ==========================================
def simulate(pid, T=20.0, dt=0.01):
    x = v = 0.0
    pid.reset()
    hist_x, hist_u, hist_d = [], [], []
    
    for i in range(int(T / dt)):
        t = i * dt
        # 5 秒时目标从 1 突变到 20
        target = 1.0 if t < 5.0 else 20.0
        
        u, d_term = pid.update(target, x, dt)
        v += u * dt 
        x += v * dt
        
        hist_x.append(x)
        hist_u.append(u)
        hist_d.append(d_term)
        
    return np.arange(int(T / dt)) * dt, hist_x, hist_u, hist_d


# ==========================================
# 验证测试
# ==========================================
print("=== 表格验证 (目标 1→20, Kp=10, Ki=5, Kd=5) ===")
for lim in [10, 20]:
    print(f"\n限幅 ±{lim}")
    print(f"{'模式':<10} | {'峰值':<8} | {'超调':<8} | {'终值':<8}")
    print("-" * 45)
    for mode in ['none', 'clamp', 'conditional']:
        pid = PID(kp=10, ki=5, kd=5, u_lim=lim, mode=mode)
        _, x_hist, _, _ = simulate(pid)
        
        peak = max(x_hist)
        final = x_hist[-1]
        overshoot = (peak - 20.0) / 20.0 * 100 
        
        mode_name = {'none': '无抗饱和', 'clamp': '钳位', 'conditional': '条件积分'}[mode]
        print(f"{mode_name:<10} | {peak:>7.3f} | {overshoot:>6.1f}% | {final:>8.3f}")

# 验证微分先行（无限幅看尖峰）
pid_classic = PID(kp=10, ki=5, kd=5, u_lim=float('inf'), derivative_on_measurement=False)
_, _, u_kick, d_kick = simulate(pid_classic)

pid_improved = PID(kp=10, ki=5, kd=5, u_lim=float('inf'), derivative_on_measurement=True)
_, _, u_no_kick, d_no_kick = simulate(pid_improved)

print("\n=== 微分冲击验证 ===")
print(f"传统微分 |D项| 最大值: {max(np.abs(d_kick)):.1f}")
print(f"微分先行 |D项| 最大值: {max(np.abs(d_no_kick)):.1f}")

# 画图
t_arr = np.arange(2000) * 0.01
plt.figure(figsize=(14, 5))

plt.subplot(1, 2, 1)
plt.plot(t_arr, u_kick, label='Classic PID (Derivative Kick)', color='red', linestyle='--')
plt.title('Control Output u (Derivative Kick is 9500!)')
plt.xlabel('Time (s)')
plt.ylabel('Control Output u')
plt.ylim(-100, 500) 
plt.legend()
plt.grid(True)

plt.subplot(1, 2, 2)
plt.plot(t_arr, d_kick, label='Classic PID D-term', color='red', linestyle='--')
plt.plot(t_arr, d_no_kick, label='Improved PID D-term', color='green', linewidth=2)
plt.title('Derivative Term Spike (9500 vs 144)')
plt.xlabel('Time (s)')
plt.ylabel('D-term Output')
plt.ylim(-100, 1000)
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()