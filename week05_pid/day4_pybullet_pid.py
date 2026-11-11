# 今天是一次巨大的跨越 把前三天在纯数学世界里写的PID代码 塞进到一个真实的物理仿真引擎PyBullet里面

# 先把前几天学的一维小车和机械臂对比一下 进行思路的转换：
# 物理量	              前三天（一维小车）	        今天（机械臂的一个关节）
# 我们控制的变量	         位置 x （米）	            关节角度 pos （弧度）
# 速度	                    v （米/秒）	              关节角速度 vel （弧度/秒）
# 我们输出的控制力	         推力 u （牛顿）	        力矩 u （牛·米 / Torque）
# 目标	                 目标位置 target = 1.0	     目标角度 target = 0.5*sin(...)
# 物理定律           	质量块受推力产生加速度	          机械臂受扭矩产生角加速度 

# 数学公式完全一模一样：
# 前三天算的是：u = kp * (目标位置 - 当前位置)
# 今天算的是：u = kp * (目标角度 - 当前角度)
# 今天唯一的变化，就是不用再自己手写 v += u*dt 和 x += v*dt 这两行代码去模拟物理世界了。 
# 把算出来的力矩 u 扔给 PyBullet，PyBullet 会自动计算质量、惯性、摩擦力、重力，然后出结果：
# “好的，下一秒机械臂转到了这个角度 pos”


# 第一部分 PyBullet的两个致命深坑 
# 在 PyBullet 里，如果你直接把你写的 PID 代码扔进去，机械臂绝对不会动。原因有两个：

# 1.PyBullet 导入的 URDF 模型，默认关节上挂着一个隐藏的速度电机。
# 它就像一个死板的保安：“只要你给的力不是用来维持指定速度的，我统统拦截”
# 解法：在开始 PID 控制前，必须写一个 for 循环，把所有关节的电机强制关闭（设为 VELOCITY_CONTROL，并且 force=0.0）
# 这就好比把保安撤了，把控制权彻底交还给 PID 算法

# 2.重力陷阱 今天你用的 two_link_arm_2.urdf 机械臂是在垂直平面（Z轴）内运动的。而重力是垂直向下的。重力方向和旋转轴平行，根本产生不了力矩！
# 今天看不到“重力导致的稳态误差”。因为今天没有重力阻力，光靠 P 就能完美控制。
# 只有换成有重力力矩的模型，才能看到 I 的真正威力

# 第二部分 读关节状态与核心代码逻辑
# pos, vel, reaction, applied = p.getJointState(arm, joint_id)

#  pos     关节角（rad）   如果做位置控制，这就是PID控制器里面的current
#  vel     关节角速度（rad/s）    如果做速度控制，这就是PID控制器里面的current
#  reaction 反作用力矩    关节对外部环境施加的力，或者环境给关节的力
#  applied  上一步施加的力矩     你上一帧调用 setJointMotorControl2 到底给电机施加了多少力

# 核心代码逻辑如下：
# pos, _, _, _ = p.getJointState(arm, joint_id)  # 1. 读（传感器读数）  看 arm 这个机械臂，它的 joint_id 号关节，现在到底转到多少度了
# 
# u = pid.update(tar, pos, dt)                   # 2. 算（大脑算力）    把第 1 步读到的 pos（我在哪），和 tar（我要去哪），以及时间 dt，喂给写好的 PID 类
#                                                 pid.update 内部会快速进行 P、I、D 的数学计算，吐出一个控制量 u。这个 u 就是大脑觉得“为了到达目标，我现在应该输出多大的力（力矩）
# p.setJointMotorControl2(..., TORQUE_CONTROL, force=float(u)) # 3. 给（踩油门）
# 
# p.stepSimulation()                             # 4. 推进物理时间（让世界过一帧）

# 第三部分 验收标准与物理天花板
# 今天的任务不是从0走到1，而是让关节“跟着一个正弦波跳舞”。
# 目标函数：target = 0.5 * sin(2 * pi * 0.1 * t) （振幅 0.5，频率 0.1Hz，周期 10 秒）

# 3.1 什么是RMS 均方根误差
# 机械臂在跟着正弦波跳舞，目标在变，实际位置也在变。有时候你超前了，有时候你落后了。
# 把每一秒你落后多少（误差），平方一下，求个平均值，再开个根号，就得到了 RMS。
# 数字含义：目标是振幅 0.5（也就是在 -0.5 到 0.5 之间摇摆）。RMS 是 0.0178，意思是你的机械臂平均比目标慢/差了 0.0178 个弧度。这个数字越小，说明你跟得越紧

# 3.2 什么是增益
# 增益”在控制领域，就是个英文翻译（Gain），它的本质意思就是“放大倍数”
# 在 PID 里，Kp, Ki, Kd 统称为增益系数（Gains）
# Kp（比例增益）：调大它，系统反应更剧烈、更敏感。调太大，机械臂就会因为过度反应而疯狂颤抖。
# Ki（积分增益）：调大它，消除微小偏差的速度越快。调太大，容易导致积分爆炸。
# Kd（微分增益）：调大它，刹车越猛。调太大，遇到一点点噪声，机械臂就会以为速度极快，瞬间大力刹车，导致卡顿

# 3.3 以下结合代码结果讲：
# 3.3.1 为什么只有 P，系统会“滞后”？（对应 RMS = 0.0178）
# 原理：P 控制（比例控制）是基于“当前误差”来出力的。误差 = 目标 - 当前
# 生活比喻：你在遛狗（狗是机械臂，你是目标）。你往前走（正弦波往上走），狗只有在发现绳子被拉紧了（产生了误差），才会往前走
# 结果：狗永远比你慢半拍。你走到最高点准备往回走了，狗才刚刚走到最高点。这就是图里说的 “当它发现偏差时，正弦波已经跑远了”
# 因为没有提前量，纯 P 的 RMS 只能做到 0.0178，在高速跟踪时，这个误差显得很大。

# 3.3.2为什么加了 I 和 D 就“超神”了？（对应 RMS = 0.0066）
# 黄金参数：Kp=80, Ki=30, Kd=3。

# Kp 变大（80）：绳子绷得更紧了，稍微差一点，狗就猛拽绳子往前走，响应变快了
# D（微分项，预测未来）：D 看的是“误差的变化速度”。如果你的目标正在快速往下掉，D 项发现“趋势不对，马上要差很多了”，它会提前往回拉（也就是提前刹车）
# I（积分项，消除余差）：负责消除那些微小的、一直存在的滞后

# 结果：有了 D 的预判，加上大 Kp 的强拉，机械臂死死咬住了目标。RMS 直接降到了 0.0066，控制效果肉眼可见地变好

# 3.3.3 最核心的坑——“物理天花板”（为什么 Kp 调到 200 也没用？）
# 这是最重要的一点，也是最反直觉的。

# 实验现象：继续加大 Kp，本以为误差会越来越小。结果 Kp 调到 200，RMS 不仅没降，反而微微升到了 0.00695
# 为什么？ 因为控制系统有“带宽（Bandwidth）”和“惯性”

# 生活比喻：你开着一辆重型卡车（机械臂），你要它跟着一辆灵活的小轿车（正弦波目标）跳舞
# 目标的周期是 10 秒（0.1Hz）。卡车勉强能跟上
# 如果给卡车换上最牛逼的发动机（Kp 调到 200），只要方向盘稍微打偏一点，卡车就猛打方向。但卡车有巨大的惯性，你猛打方向盘会导致轮胎打滑（系统振荡），它并不会立刻瞬移到目标位置
# 电机的最大力矩是有限的（物理上限），机械臂有重量，它不可能瞬间加速到无限大。

# 什么是“物理地板”？
# 0.0066 就是这个机械臂在这个速度下的极限
# 你想降得更低？可以！ 把正弦波的频率降下来，从 0.1Hz 降到 0.05Hz（周期 20 秒）。目标晃得越慢，机械臂越容易跟
# 如果你非要在 0.1Hz 的速度下把误差压下去，光靠调 PID 是没用的，你必须换一个更强壮的电机（力矩更大）、换一个更轻的连杆（减少惯性），或者提前给一个“前馈”控制（就像你先知道前面有坑，提前打好方向盘，而不是等掉进去再打）

# 总结一句话：PID 调参不是万能的。你可以把参数调到最优，但永远无法突破执行机构（电机、机械结构）的物理极限

# 第四部分 完整代码实操：
import numpy as np
import pybullet as p
import pybullet_data
import os

# ⚠️ 核心修改 1：强制 Matplotlib 使用无图形界面的后端（只存图，不弹窗）
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ==========================================
# 1. PID 类（保持不变）
# ==========================================
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, u_lim=50.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.u_lim = u_lim
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_val = None
        self.prev_error = 0.0

    def update(self, target, current, dt):
        e = target - current
        if self.prev_val is None:
            de = 0.0
        else:
            de = -(current - self.prev_val) / dt   # 微分先行 对测量值求导
        
        self.prev_error = e
        self.prev_val = current  # 这两行是更新记忆

        raw = self.kp * e + self.ki * self.integral + self.kd * de   # 理论计算输出
        u = max(-self.u_lim, min(self.u_lim, raw))   # u：实际物理输出。用 max 和 min 强行截断在 ±u_lim 之间，这叫限幅

        if u == raw:
            self.integral += e * dt
            # 条件积分抗饱和：只有当 u 等于 raw（说明电机没撞到天花板，还有余力）时，才允许累加积分
            # 如果饱和了，就不记账了，防止积分爆炸
        return u

# ==========================================
# 2. 封装跑数据的函数
# ==========================================
def run_simulation(pid, arm, joint_id, target_fn, T=8.0, dt=1/240):  # 输入PID类 机械臂 关节编号 目标函数 总时间 时间步长
    pid.reset()

    # ⚠️ 关键：pid.reset() 只清【控制器自己的记忆】（积分器、上次误差），
    #    机械臂的【物理状态】一点没动 —— 还停在上一轮的终点。
    #    所以要把【所有关节】都归零：
    #      只归零被测的 joint_id 还不够，因为两连杆的质量矩阵是耦合的，
    #      关节 1 的角度会改变关节 0 的等效惯量。
    for j in range(p.getNumJoints(arm)):
        p.resetJointState(arm, j, 0.0, 0.0)

    ts, tars, poss, us = [], [], [], []

    for k in range(int(T / dt)):
        t = k * dt
        tar = target_fn(t)   # 这个时刻的正弦波目标 tar
        
        pos, _, _, _ = p.getJointState(arm, joint_id)   # 从引擎获取真实的关节角度 pos
        u = pid.update(tar, pos, dt)   # 把目标、实际位置、时间步喂给大脑，算出力矩 u
        
        p.setJointMotorControl2(arm, joint_id, p.TORQUE_CONTROL, force=float(u))   # 用纯力矩模式输出
        p.stepSimulation()   # 让物理引擎往前走 dt（1/240 秒）
        
        ts.append(t)
        tars.append(tar)
        poss.append(pos)
        us.append(u)
        
    rms = np.sqrt(np.mean((np.array(tars) - np.array(poss))**2))   # 计算 RMS（均方根误差）
    return ts, tars, poss, us, rms   # 时间  目标角度  真实实际物理角度  记录力矩  均方根误差

# ==========================================
# 3. 主程序：跑两组数据并保存图片
# ==========================================
if __name__ == "__main__":
    print("1. 正在初始化 PyBullet...")
    p.connect(p.DIRECT) # 纯后台跑数据
    
    HERE = os.path.dirname(os.path.abspath(__file__))
    URDF = os.path.join(HERE, "../week03_fk/day3_two_link_arm_2.urdf") 
    if not os.path.exists(URDF):
        URDF = pybullet_data.getDataPath() + "/kuka_iiwa/model.urdf"

    arm = p.loadURDF(URDF, useFixedBase=True)
    for j in range(p.getNumJoints(arm)):
        p.setJointMotorControl2(arm, j, p.VELOCITY_CONTROL, force=0.0)   # 把模型自带的“速度电机”全部关掉

    target_fn = lambda t: 0.5 * np.sin(2 * np.pi * 0.1 * t)   # 定义目标：幅度 0.5，频率 0.1Hz 的正弦波。
    joint_id = 0   # 指定要控制的关节

    print("2. 正在跑纯 P 控制 (Kp=20)...")
    pid_p = PID(kp=20, ki=0, kd=0)
    ts_p, tars_p, poss_p, us_p, rms_p = run_simulation(pid_p, arm, joint_id, target_fn)   # 输入参数 得到输出
    
    print("3. 正在跑黄金 PID 控制 (Kp=80, Ki=30, Kd=3)...")
    pid_pid = PID(kp=80, ki=30, kd=3)
    ts, tars, poss, us, rms_pid = run_simulation(pid_pid, arm, joint_id, target_fn)

    p.disconnect() # 算完直接关闭引擎

    # 打印终端结果
    print("\n" + "="*40)
    print(f"✅ 纯 P 控制 RMS: {rms_p:.4f}  (要求 ≈ 0.0178)")
    print(f"✅ 黄金 PID RMS: {rms_pid:.4f} (要求 ≈ 0.0066)")
    print("="*40 + "\n")

    # ==========================================
    # 4. 画图并保存
    # ==========================================
    print("4. 正在生成图表...")
    plt.figure(figsize=(14, 5))
    
    # 左图：位置跟踪对比
    plt.subplot(1, 2, 1)
    plt.plot(ts, tars, 'k--', label='Target', linewidth=2)
    plt.plot(ts_p, poss_p, color='orange', alpha=0.7, label=f'Pure P (RMS: {rms_p:.4f})')
    plt.plot(ts, poss, 'r', label=f'Gold PID (RMS: {rms_pid:.4f})', linewidth=2)
    plt.title('Target vs Actual (P vs PID)')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.legend()
    plt.grid(True)

    # 右图：控制力矩对比
    plt.subplot(1, 2, 2)
    plt.plot(ts_p, us_p, color='orange', alpha=0.7, label='Pure P Output')
    plt.plot(ts, us, 'r', label='Gold PID Output')
    plt.title('Control Output (u)')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    
    # ⚠️ 核心修改 2：直接保存为图片，不弹窗
    img_path = os.path.join(HERE, "day4_result.png")
    plt.savefig(img_path, dpi=300, bbox_inches='tight')
    
    print(f"5. ✅ 任务完成！图表已成功保存至：\n 👉 {img_path}")