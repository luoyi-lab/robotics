# 在一个纯python的一维小车模型上看清PID各自干什么
# 目标设定 让一个质量快（小车）从位置0走到位置1

# (1) P(Proportional) 比例项
# 公式：u = Kp * e
# 推力 = 比例系数 × 距离目标还有多远
# 离得远，就用力推；离得近，就轻轻推 就像弹簧一样

# (2) D(Derivative) 微分项
# 公式：u = Kd * de/dt
# 推力 = 阻尼系数 × 误差变化的快慢（即速度）
# P 管你差多远，D 管你跑多快。你虽然离终点还有一点距离，但你冲得太快了，D 就会提前踩刹车，防止你冲过头。
# 作用：把振荡压下去，类似于刹车片

# (3) I(Integral) 积分项
# 公式：u = Ki * ∫e dt
# 推力 = 积分系数 × 过去所有误差的累积
# 比如地面有摩擦力，P 觉得差 0.05 太小了，不给力。I 就会把过去每一秒差的 0.05 加起来，越加越大，直到力大到足以克服摩擦力
# 作用：消除稳态误差 解决“永远差一点点到不了终点”的问题
# 副作用：【积分饱和】。
# 因为它是累加的，如果你推了很久都没到（或者你在推的过程中被卡住了），I 项就会攒出一个超级无敌巨大的力。
# 等你突然到了终点，这个巨大的力还没来得及消退，就会一把把你推飞出去

# 如果只有P 会一直振荡
# 用一个没有摩擦力的秋千来代入：
# 起步（0 → 0.9）：你（P）看到差了 1 米，拼命推车。车越来越快，积累了很大的动能。
# 快到终点（0.9 → 1.0）：此时误差 e 变小了，P 给你的力也变小了（公式里 u = Kp * e）。但是！因为质量块有惯性，它带着之前攒下的速度，根本刹不住车，直接冲过去了。

# 冲过头（1.0 → 1.1）：车到了 1.1 的位置。此时误差变成了负数（目标 1 减去当前 1.1）。P 开始反向用力，试图把车拉回来。车子冲过去时带有的动能，被 P 的反向力转化为了“势能”。

# 反向冲回（1.1 → 0）：势能再次转化为动能，车又带着速度冲回了起点。

# 结论： 在这个过程中，没有东西消耗能量（没有摩擦力，没有空气阻力）。根据能量守恒定律，它能冲到多高，就一定能荡回来多远。只要没有东西把能量消耗掉，它就会像正弦波一样，永远在 1 的左右来回振荡，永不停息。

# D 项就是在消耗这个能量！
# D 项（微分）天生是跟“速度”作对的。速度越快，D 给你的反向力就越大。它把你本用来“冲过头”的动能，通过反向刹车消耗掉了。有了 D，车才不会冲过头，振荡的幅度才会一波比一波小，最终安安静静地停在终点 1。

# 一句话总结这张图：
# P 是油门（容易冲过头），D 是刹车（防止冲过头），I 是死磕（哪怕差一点也要干到底）

# P 是“油门”负责快，D 是“刹车”负责稳，I 是“恒心”负责准
# P 负责接近，D 负责防过冲，I 负责消除最后的差距，三者配合与制衡，最终实现“又快又稳又准”

# 代码训练：
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import platform

# ==========================================
# 1. 解决 VSCode 下 Matplotlib 中文乱码问题
# ==========================================
def set_chinese_font():
    """找一个当前环境里真有的中文字体。找不到就明确说出来，而不是静默变方块。"""
    candidates = [
        'Microsoft YaHei', 'SimHei', 'DengXian', 'SimSun',   # Windows
        'PingFang SC', 'Heiti TC', 'Arial Unicode MS',       # macOS
        'WenQuanYi Micro Hei', 'Noto Sans CJK SC',           # Linux
    ]
    available = {f.name for f in fm.fontManager.ttflist}     # ← 直接查已加载的字体表
    for name in candidates:
        if name in available:
            plt.rcParams['font.sans-serif'] = [name, 'DejaVu Sans']
            plt.rcParams['axes.unicode_minus'] = False
            print(f'[font] 使用中文字体: {name}')
            return
    print('[font] ⚠️ 没找到中文字体，中文会显示成方块')

set_chinese_font()


# ==========================================
# 2. 仿真核心函数
# ==========================================
def simulate(control_type, Kp=10, Kd=5, Ki=5, dt=0.01, T=5.0, disturbance=0.0):
    """
    一维质量块仿真
    control_type: 'P', 'PD', 'PI', 'PID'
    disturbance: 恒定扰动（模拟重力/阻力）
    """
    steps = int(T / dt)
    x, v = 0.0, 0.0       # 初始状态：位置 0，速度 0
    target = 1.0          # 目标位置
    integral = 0.0        # 积分项累加器
    e_prev = target - x   # 上一次的误差（用于计算微分）
    
    history_x = []
    time_array = np.arange(steps) * dt
    
    for _ in range(steps):
        e = target - x
        de = (e - e_prev) / dt  # 误差变化率 (速度)
        integral += e * dt      # 误差积分 (累积)
        
        # 计算控制量 u
        if control_type == 'P':
            u = Kp * e
        elif control_type == 'PD':
            u = Kp * e + Kd * de
        elif control_type == 'PI':
            u = Kp * e + Ki * integral
        elif control_type == 'PID':
            u = Kp * e + Kd * de + Ki * integral
            
        # 动力学方程：a = u (m=1) - disturbance (阻力)
        a = u - disturbance
        
        # 欧拉积分更新状态
        v += a * dt  # 速度 = 之前的速度 + 加速度 × 时间
        x += v * dt  # 位置 = 之前的位置 + 速度 × 时间
        
        history_x.append(x)
        e_prev = e 
        
    return time_array, history_x


# ==========================================
# 3. 图 1：四种控制律对比
# ==========================================
t, x_p = simulate('P')
_, x_pd = simulate('PD')
_, x_pi = simulate('PI')
_, x_pid = simulate('PID')

plt.figure(figsize=(10, 5))
plt.plot(t, x_p, label='① P (等幅振荡)', linestyle='--')
plt.plot(t, x_pd, label='② PD (振荡衰减)')
# 【修改点 ②】：把“超调”改成“无阻尼，发散”
plt.plot(t, x_pi, label='③ PI (无阻尼，发散)') 
plt.plot(t, x_pid, label='④ PID (又快又稳)', linewidth=2)
plt.axhline(1.0, color='black', linewidth=1, linestyle=':', label='目标位置')
plt.title('四种控制律画在一张图上', fontsize=14)
plt.xlabel('时间 (s)')
plt.ylabel('位置 x')
plt.legend()
plt.grid(True)


# ==========================================
# 4. 图 2：验证实验（加入扰动，证明 I 的价值）
# ==========================================
# 【修改点 ①】：图 2 换成 PD vs PID，时间改成 15 秒
# 注意：这里修正了你原代码里的 bug，必须用 t15 接收时间轴，否则下面画图会报错
t15, x_pd_dist  = simulate('PD',  T=15.0, disturbance=0.5)
_, x_pid_dist = simulate('PID', T=15.0, disturbance=0.5)

plt.figure(figsize=(10, 5))
plt.plot(t15, x_pd_dist,  label='PD  + 恒定扰动', linestyle='--', color='orange')
plt.plot(t15, x_pid_dist, label='PID + 恒定扰动', color='green', linewidth=2)
plt.axhline(1.0,  color='black', linewidth=1, linestyle=':', label='目标位置 (1.0)')
plt.axhline(0.95, color='gray',  linewidth=1, linestyle='--', label='PD 稳态 0.95')
plt.title('为什么需要 I：消除稳态误差', fontsize=14)
plt.xlabel('时间 (s)')
plt.ylabel('位置 x')
plt.legend()
plt.grid(True)


# ==========================================
# 5. 第 165 行注释补充（思维总结）
# ==========================================
# I (积分) 就是“强迫症”：为了消除稳态误差而生。哪怕只差 0.05，只要一直积累，也能把力撑大。
# ⚠️ 但前提是系统【有阻尼】—— 光有 P+I 在纯质量块上会发散（见图 1 的绿线）。
# 图 2 用 PD vs PID 才能看到“PD 停在 0.95，PID 爬到 1.0”的真实效果。
# 因为积分项爬得很慢，在 Ki=5 的参数下，T=5 时 PID 只到 1.016，需要 T=15 才能稳稳到达 1.000000。


# 在 VSCode 中弹出独立窗口显示图片
plt.show()

# 这项工作到底是要学会啥：
# P（比例）就是弹簧：差多远，给多大劲。让你知道系统动起来的最基本动力来源。但只有弹簧，物体就会像秋千一样永远晃。

# D（微分）就是刹车/减震器：看速度给阻力。让你亲眼看到，只要加上 D，振幅就会肉眼可见地衰减，最终稳定下来。（橙线展现的）

# I（积分）就是“强迫症”：为了消除稳态误差而生。哪怕只差 0.05，只要一直积累，也能把力撑大。（图 2 绿线在稳定后，慢慢爬向 1 的原因）
# 理解系统稳定到底有多难 纯PI会发散 D项是保命的

# 思考题（明天用）
# (1)只有 P 时，Kp 调大一倍，振荡更快还是更慢？幅度更大还是更小？
# 振荡更快 幅度理论上不变
# Kp调大一倍，变成 2Kp，角频率 ω 会变成原来的 根号2倍
# 幅度为什么不变 因为系统能量守恒 来回的极限距离永远不会变

# (2)只有 P + 恒定扰动时，稳态误差和 Kp 是什么关系？（提示：稳态时 u 必须等于扰动）
# e=扰动/Kp 二者成反比，Kp 越大，误差越小
# 物理视角：受力平衡 即 u=扰动力 故：Kp⋅e 稳态 = 扰动力

# (3)Kd 调太大会怎样？（提示：想想噪声）
# 系统会剧烈高频抖动（甚至烧毁执行机构），因为 D 项会把传感器噪声放大成灾难