# 前几天 都是只控制一个小车或者控制一个关节 今天要同时控制两个关节 要体会到物理世界里的的机械结构 是会互相打架的

# 第一部分 代码的克隆与并行
# 任务是让机械臂的两根连杆 各自按不同的节奏跳舞 用同一个pid类 也是同一套run_simulation 但是有两个对象
# 关节 0（大臂）：慢慢摇，频率 0.1Hz，幅度 0.5
# 关节 1（小臂）：摇得快一点，频率 0.15Hz，幅度 0.8
# 雇两个大脑：pid0 = PID(...) 和 pid1 = PID(...)
# 在同一个主循环里面同时指挥他们：
# 1. 读关节 0 -> 算关节 0 -> 给关节 0 力矩
# pos0 = p.getJointState(...)
# u0 = pid0.update(...)
# p.setJointMotorControl2(...)

# 2. 读关节 1 -> 算关节 1 -> 给关节 1 力矩
# pos1 = p.getJointState(...)
# u1 = pid1.update(...)
# p.setJointMotorControl2(...)

# 3. 统一推进时间（让物理世界走一帧）
# p.stepSimulation()

# 两个大坑要牢记：(1) 要关闭所有默认电机 (2) 每次实验要重置所有关节的物理状态

# 第二部分 核心物理实验：关节耦合现象
# 这是今天最值钱的硬核知识。实验结果发现了一个非常反直觉的结论：单向耦合

# 实验设置：
# 实验 A：把关节 0 强行锁死在 0 度（目标一直是 0），只让关节 1 疯狂摇摆（目标正弦波）
# 现象：关节 0 原本应该纹丝不动，结果误差 RMS0 居然从 0 涨到了 0.00638！它被推偏了！

# 实验 B：关节 0 和 关节 1 一起摇摆。
# 现象：关节 1 的误差几乎没变（只涨了 5%），它根本不在乎关节 0 怎么动。

# 为什么是单向的？
# 想象你的手抓着两根连接在一起的棍子
# 关节 0 是靠近你手的棍子（近端）
# 关节 1 是远处的棍子（远端）

# 远端动，近端遭殃（实验 A）：
# 当你狂甩远端（关节 1）时，根据牛顿第三定律（作用力与反作用力），关节 1 会产生巨大的离心力和反作用力矩。这股力量顺着连杆，硬生生地推拽你的手腕（关节 0）
# 关节 0 的 PID 遭遇了狂暴的外部干扰力矩。它原本以为目标不变，可以歇着，结果被这股反作用力打了个措手不及。PID 的响应是有延迟的，它只能事后补救，所以误差（RMSE0）被硬生生扯出了

# 近端动，远端躺平（实验 B）：
# 当你转动近端（关节 0）时，远端（关节 1）相当于被人端着底座平移了。关节 1 的 PID 只需要控制小臂相对于大臂的夹角，能立刻出力修正，轻松跟上。所以关节 1 几乎感受不到干扰

# 结论：
# 在低速、轻载时，你用两个独立的 PID 各自为战，是没问题的。
# 但在高速、重载（比如工业机械臂疯狂挥舞）时，这种互相干扰的力学耦合会极其致命。这时候你必须要学更高级的动力学控制（计算力矩法、WBC 全身控制），把这种干扰提前算出来并抵消掉

# 第三部分 调参与交差
# 1. 玄学调参（图4）
# 这部分没有公式，只有“手感” 把规律变成肌肉记忆：

# Kp 调大：力量更猛，跟得更紧（误差小），但到了极限会剧烈晃动
# Ki 调大：能强行消除最后的一丝误差，但极容易“冲过头”（超调）
# Kd 调大：刹车更猛，能压住晃动。但你看力矩曲线图，它会变得像心电图一样“长毛”（高频抖动）。因为 D 项对速度极其敏感，物理引擎的一点点微小震动，都会被它放大成巨大的反向力矩，导致电机疯狂抽搐
# 验证方法：固定另外两个参数，只动一个，看 RMS 和力矩曲线怎么变

# 2. 录制 GIF 动画（图5）
# 不需要改底层逻辑，只需要在 stepSimulation() 之后，每隔几帧用 p.getCameraImage() 抓一张画面存起来。
# 收集了几百张画面后，用 imageio.mimsave('demo.gif', frames) 把图片打包成动图。
# 注意：GIF 录制时，物理时间流逝和画面抓取的时间需要配合好，否则录出来的动作会像快进。

# 完整代码如下：
import numpy as np
import pybullet as p
import pybullet_data
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image   # 用 PIL 存 GIF —— 环境里没有 imageio，W3 Day6 用的就是 PIL

# ==========================================
# 0. GIF 录制参数
# ==========================================
W, H = 640, 480             # 画面分辨率
CAPTURE_EVERY = 10          # 每 10 个仿真步抓一帧 → 240/10 = 24 fps
GIF_DURATION_MS = int(1000 * CAPTURE_EVERY / 240)   # 41 ms → 和物理时间一致，不会快进


# ==========================================
# 1. PID 类（复用）
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
            de = -(current - self.prev_val) / dt  # 微分先行
        self.prev_error = e
        self.prev_val = current  # 记录位置

        raw = self.kp * e + self.ki * self.integral + self.kd * de
        u = max(-self.u_lim, min(self.u_lim, raw))

        if u == raw:    # 条件积分抗饱和：只有当输出没有被限幅（u == raw）时，才允许累加误差
            self.integral += e * dt
        return u

# ==========================================
# 2. 两关节仿真与录制引擎
# ==========================================
def run_simulation(arm, T=8.0, dt=1/240, record_gif=False):
    # 实例化两个大脑
    # joint0 扛着整条手臂（惯量大），Ki 太大会让它起步慢振荡 —— 实测降到 Ki=10 更干净
    pid0 = PID(kp=125, ki=10, kd=10)   # 调参就是这里
    # joint1 只扛小臂（惯量小），用原来那组就行
    pid1 = PID(kp=80, ki=30, kd=3)   # 调参就是这里
    pid0.reset()
    pid1.reset()

    # 初始化物理状态（归零所有关节）
    for j in range(p.getNumJoints(arm)):
        p.resetJointState(arm, j, 0.0, 0.0)

    ts, tars0, poss0, us0 = [], [], [], []
    tars1, poss1, us1 = [], [], []

    # GIF 录制准备
    frames = []
    if record_gif:
        # ⚠️ DIRECT 模式没有"调试相机"，resetDebugVisualizerCamera 在这里是【空操作】。
        #    必须自己算 view / projection 矩阵传进去，否则拍出来是默认视角（往往拍不到机械臂）。
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0.9, 0, 0.3],
            distance=2.2, yaw=50, pitch=-26, roll=0, upAxisIndex=2)
        proj = p.computeProjectionMatrixFOV(fov=60, aspect=W / H,
                                            nearVal=0.1, farVal=100)

    for k in range(int(T / dt)):
        t = k * dt
        
        # 目标：关节 0 慢（0.1Hz），关节 1 快（0.15Hz）
        tar0 = 0.5 * np.sin(2 * np.pi * 0.10 * t) 
        tar1 = 0.8 * np.sin(2 * np.pi * 0.15 * t)

        # --- 关节 0 ---
        pos0, _, _, _ = p.getJointState(arm, 0)
        u0 = pid0.update(tar0, pos0, dt)
        p.setJointMotorControl2(arm, 0, p.TORQUE_CONTROL, force=float(u0))

        # --- 关节 1 ---
        pos1, _, _, _ = p.getJointState(arm, 1)
        u1 = pid1.update(tar1, pos1, dt)
        p.setJointMotorControl2(arm, 1, p.TORQUE_CONTROL, force=float(u1))

        # --- 推进物理 ---
        p.stepSimulation()

        # 记录数据
        ts.append(t)
        tars0.append(tar0); poss0.append(pos0); us0.append(u0)
        tars1.append(tar1); poss1.append(pos1); us1.append(u1)

        # 录制动图（每隔 CAPTURE_EVERY 步抓一帧）
        if record_gif and k % CAPTURE_EVERY == 0:
            _, _, rgbImg, _, _ = p.getCameraImage(
                W, H, view, proj, renderer=p.ER_TINY_RENDERER)
            frames.append(Image.fromarray(np.uint8(rgbImg[:, :, :3])))

    # 保存 GIF
    if record_gif:
        gif_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "day5_pid_demo.gif")
        frames[0].save(gif_path, save_all=True, append_images=frames[1:],
                       duration=GIF_DURATION_MS, loop=0)
        print(f"✅ GIF 已保存至: {gif_path}  "
              f"({len(frames)} 帧, {len(frames)*GIF_DURATION_MS/1000:.1f} 秒, "
              f"{os.path.getsize(gif_path)//1024} KB)")

    # 计算 RMS
    rms0 = np.sqrt(np.mean((np.array(tars0) - np.array(poss0))**2))
    rms1 = np.sqrt(np.mean((np.array(tars1) - np.array(poss1))**2))

    return ts, tars0, tars1, poss0, poss1, us0, us1, rms0, rms1

# ==========================================
# 3. 主程序
# ==========================================
if __name__ == "__main__":
    p.connect(p.DIRECT) # 后台跑数据
    # p.connect(p.GUI)  # 如果你想看实时画面，换成这个，并在画图代码前加 input() 挂起

    HERE = os.path.dirname(os.path.abspath(__file__))
    URDF = os.path.join(HERE, "../week03_fk/day3_two_link_arm_2.urdf") 
    if not os.path.exists(URDF):
        URDF = pybullet_data.getDataPath() + "/kuka_iiwa/model.urdf"

    arm = p.loadURDF(URDF, useFixedBase=True)

    # 加一块地面当参照物，否则 GIF 里啥也看不出来（纯白背景）
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # ⚠️ 关默认电机
    for j in range(p.getNumJoints(arm)):
        p.setJointMotorControl2(arm, j, p.VELOCITY_CONTROL, force=0.0)

    print("正在仿真，并录制 GIF...")
    # 开启 GIF 录制
    ts, tars0, tars1, poss0, poss1, us0, us1, rms0, rms1 = run_simulation(arm, record_gif=True)

    p.disconnect()

    print(f"✅ 关节 0 RMS: {rms0:.4f} (目标 < 0.03)")
    print(f"✅ 关节 1 RMS: {rms1:.4f} (目标 < 0.03)")

    # ==========================================
    # 4. 画图
    # ==========================================
    plt.figure(figsize=(14, 8))
    
    # 关节 0 的跟踪
    plt.subplot(2, 2, 1)
    plt.plot(ts, tars0, 'k--', label='Target 0')
    plt.plot(ts, poss0, 'r', label=f'Actual 0 (RMS: {rms0:.4f})')
    plt.title('Joint 0 Tracking')
    plt.legend(); plt.grid(True)

    # 关节 1 的跟踪
    plt.subplot(2, 2, 2)
    plt.plot(ts, tars1, 'k--', label='Target 1')
    plt.plot(ts, poss1, 'b', label=f'Actual 1 (RMS: {rms1:.4f})')
    plt.title('Joint 1 Tracking')
    plt.legend(); plt.grid(True)

    # 关节 0 的力矩
    plt.subplot(2, 2, 3)
    plt.plot(ts, us0, 'r')
    plt.title('Joint 0 Torque')
    plt.grid(True)

    # 关节 1 的力矩
    plt.subplot(2, 2, 4)
    plt.plot(ts, us1, 'b')
    plt.title('Joint 1 Torque')
    plt.grid(True)

    plt.tight_layout()
    img_path = os.path.join(HERE, "day5_result.png")
    plt.savefig(img_path, dpi=300)
    print(f"✅ 数据图表已保存至: {img_path}")