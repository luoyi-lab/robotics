# 2 为什么说"e_prev初始化"是最重要的一课
# 这是你 W3 Day 5 踩过的同一个坑，换了个马甲。

# 假设一开始，目标在 1，当前在 0，误差 e = 1。
# 如果你的 prev_error 初始化为 0，第一次循环算微分时：
# de = (1 - 0) / 0.01 = 100。
# 此时如果 Kd=5，那么 D 项的输出就是 5 * 100 = 500！
# 这就是“凭空制造的巨大冲击”。机器人的真实速度从 0 起步，不可能瞬间产生 500 的力，这纯粹是数学计算在代码里的漏洞。

# 和 W3 的低通滤波器对照
#                 W3 Day 5 低通	                        W5 Day 2 PID
#   现象	 信号在 100，第一拍输出 9.09	       误差是 1，第一步 de = 100
#   后果	     机器人猛冲	                              执行器猛冲
#   根因	 y 从 0 开始，没预热	                e_prev 从 0 开始，没预热
#   解法 initialized_ 标志，第一拍直接取输入	  first_call_ 标志，第一拍让 de = 0
# 共同点：凡是「带记忆」的模块，第一次调用前都必须用真实值预热

# 3 4 代码训练
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import platform

# 设置中文字体（同Day 1）
def set_chinese_font():
    candidates = ['Microsoft YaHei', 'SimHei', 'PingFang SC', 'WenQuanYi Micro Hei']
    available = {f.name for f in fm.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams['font.sans-serif'] = [name]
            plt.rcParams['axes.unicode_minus'] = False
            return
set_chinese_font()

# ==========================================
# 核心：Day 2 的 PID 类
# ==========================================
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0):  # self. 代表这是这个对象私有的
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0  # 初始化积分器为 0
        self.prev_error = None  # ← 用 None 表示“还没预热过”

    def update(self, target, current, dt):  # 这是每次循环（比如每 10ms）都要调用一次的核心函数
        e = target - current
        
        # 积分
        self.integral += e * dt
        
        # 微分（第一次调用时 de 取 0，避免凭空冲击）
        if self.prev_error is None:
            de = 0.0
        else:
            de = (e - self.prev_error) / dt
            
        self.prev_error = e  # 把这次的误差存起来，留给下一次循环用
        return self.kp * e + self.ki * self.integral + self.kd * de

    def reset(self):  # 重置就是把记忆洗掉
        """切换任务/重新启动时，把记忆清空"""
        self.integral = 0.0
        self.prev_error = None


# ==========================================
# 任务 1：用类复现 Day 1 的四条曲线
# ==========================================
def simulate(pid, T=5.0, dt=0.01, disturbance=0.0):  # def simulate(pid, ...): 这是一个被控对象仿真引擎
    x = v = 0.0  # 初始化物理状态 位置速度都为0
    pid.reset() # 每次仿真前必须重置记忆！
    hist = []  # 拿个空本子准备记录小车每一刻的位置
    
    for _ in range(int(T / dt)):  # 根据总时间 T 和步长 dt，决定循环多少次
        u = pid.update(target=1.0, current=x, dt=dt)  # 调用大脑。把目标（1.0）、现在的位置（x）、时间步长（dt）喂给刚才造出来的 PID 对象，算出控制力 u
        v += (u - disturbance) * dt  # 控制力减去阻力得到加速度，加速度乘时间得到速度（默认m=1）
        x += v * dt  # 速度乘时间得到位置
        hist.append(x)  # 记录新位置
        
    return hist

# 跑四条曲线
T = 5.0
h_p   = simulate(PID(kp=10), T=T) # 造P
h_pd  = simulate(PID(kp=10, kd=5), T=T)  # 造PD
h_pi  = simulate(PID(kp=10, ki=5), T=T)  # 造PI
h_pid = simulate(PID(kp=10, ki=5, kd=5), T=T)  # 造PID

# 以下是画图代码
t_arr = np.arange(len(h_p)) * 0.01
plt.figure(figsize=(10, 5))
plt.plot(t_arr, h_p, '--', label='① P')
plt.plot(t_arr, h_pd, label='② PD')
plt.plot(t_arr, h_pi, label='③ PI (无阻尼，发散)')
plt.plot(t_arr, h_pid, linewidth=2, label='④ PID')
plt.axhline(1.0, color='black', linestyle=':')
plt.title('Day 2 验证：用 Class 复现 Day 1 的四种曲线')
plt.legend()
plt.grid(True)
plt.show()


# ==========================================
# 任务 2：dt 到底能有多大？
# ==========================================
print("\n--- dt 稳定性实验 (Kp=10, Ki=5, Kd=5) ---")
dt_list = [0.01, 0.1, 0.3, 0.5]  # 准备好要测试的步长
for dt in dt_list:
    pid = PID(kp=10, ki=5, kd=5) 
    hist = simulate(pid, T=10.0, dt=dt)
    
    # 打印峰值和终值
    peak = max(hist)  # 记录小车最高冲到了哪里（看超调）
    final = hist[-1]  # 记录 10 秒结束那一刻，小车在哪里
    status = "稳定" if abs(final) < 100 else "❌ 发散"
    print(f"dt = {dt:>4}: 峰值 = {peak:8.3f}, 终值 = {final:10.3f} | {status}")


# 阻尼项稳定性：Kd · dt < 2
# 弹簧项稳定性：dt < 2 / √Kp

print("\n--- 再试一组：Kp=20 (增益变大) ---")
# 理论极限：Kd*dt < 2 -> dt < 0.4 (Kd=5)；dt < 2 / sqrt(20) ≈ 0.447
# 取交集，dt < 0.4 稳定，0.4 及以上发散。所以 dt=0.2 会放大但收敛，dt=0.3 也会收敛。
dt_list_2 = [0.05, 0.1, 0.2]
for dt in dt_list_2:
    pid = PID(kp=20, ki=5, kd=5)
    hist = simulate(pid, T=10.0, dt=dt)
    print(f"dt = {dt:>4}: 终值 = {hist[-1]:8.3f}")

# 结论：dt 的上限由增益决定，不是随便定的。
# 真实机器人上，控制周期一旦抖动（dt 不恒定），D 项就会失真 —— 这是 Day 3「微分先行」的伏笔