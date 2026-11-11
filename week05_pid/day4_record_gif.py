# ============================================================
# W5 Day 4 · 无窗口录制：DIRECT 模式跑 PID，直接存成 GIF
#
#   为什么要这个文件？
#   你的机器上 PyBullet GUI 窗口和 matplotlib 窗口都弹不出来，
#   但【不需要窗口也能出图】—— 用 getCameraImage 在后台拍帧，
#   再用 PIL 拼成 GIF。这条路不依赖任何显示服务器。
#
#   而且：里程碑① 要的就是「10 秒演示视频」—— 这个文件直接产出它。
#
#   运行：  ~/pybullet-env/bin/python day4_record_gif.py
#   产物：  week05_pid/day4_pid_demo.gif
# ============================================================
import os

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

from matplotlib import pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "../week03_fk/day3_two_link_arm_2.urdf")
GIF = os.path.join(HERE, "day4_pid_demo.gif")
PNG = os.path.join(HERE, "day4_pid_curve.png")

# ---------- 画面 ----------
W, H = 640, 480
FPS = 25
N_FRAMES = 250                 # 250 / 25 = 10.0 秒
SIM_DT = 0.004                 # 250 Hz
STEPS_PER_FRAME = 10           # 250Hz / 25fps = 10

# ---------- 控制 ----------
KP, KI, KD = 80.0, 30.0, 3.0
U_LIM = 50.0
JOINT = 0


class PID:
    """微分先行 + 条件积分"""

    def __init__(self, kp, ki=0.0, kd=0.0, u_lim=50.0):
        self.kp, self.ki, self.kd, self.u_lim = kp, ki, kd, u_lim
        self.integral = 0.0
        self.prev_val = None

    def update(self, target, current, dt):
        e = target - current
        de = 0.0 if self.prev_val is None else -(current - self.prev_val) / dt
        self.prev_val = current
        raw = self.kp * e + self.ki * self.integral + self.kd * de
        u = max(-self.u_lim, min(self.u_lim, raw))
        if u == raw:
            self.integral += e * dt
        return u


def main():
    # ---------- 1. 后台起引擎 ----------
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    arm = p.loadURDF(URDF, useFixedBase=True)

    # 关掉默认速度电机
    for j in range(p.getNumJoints(arm)):
        p.setJointMotorControl2(arm, j, p.VELOCITY_CONTROL, force=0.0)

    # ---------- 2. 相机 ----------
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.6, 0, 0.5],
        distance=2.6, yaw=50, pitch=-22, roll=0, upAxisIndex=2)
    proj = p.computeProjectionMatrixFOV(fov=60, aspect=W / H,
                                        nearVal=0.1, farVal=100)

    # ---------- 3. 边跑 PID 边拍帧 ----------
    pid = PID(KP, KI, KD, U_LIM)
    frames, ts, tars, poss, us = [], [], [], [], []
    t = 0.0

    for i in range(N_FRAMES):
        for _ in range(STEPS_PER_FRAME):
            target = 0.5 * np.sin(2 * np.pi * 0.1 * t)
            pos, _, _, _ = p.getJointState(arm, JOINT)
            u = pid.update(target, pos, SIM_DT)

            p.setJointMotorControl2(arm, JOINT, p.TORQUE_CONTROL, force=float(u))
            p.stepSimulation()

            ts.append(t); tars.append(target); poss.append(pos); us.append(u)
            t += SIM_DT

        _, _, rgb, _, _ = p.getCameraImage(W, H, view, proj,
                                           renderer=p.ER_TINY_RENDERER)
        arr = np.array(rgb, dtype=np.uint8).reshape(H, W, 4)[:, :, :3]
        frames.append(Image.fromarray(arr))
        if (i + 1) % 50 == 0:
            print(f"  已渲染 {i+1}/{N_FRAMES}")

    p.disconnect()

    # ---------- 4. 存 GIF ----------
    frames[0].save(GIF, save_all=True, append_images=frames[1:],
                   duration=int(1000 / FPS), loop=0)
    print(f"\n✅ 动画：{GIF}  ({os.path.getsize(GIF)//1024} KB, "
          f"{len(frames)} 帧, {len(frames)/FPS:.1f} 秒)")

    # ---------- 5. 存曲线图（不弹窗）----------
    ts, tars, poss, us = map(np.asarray, (ts, tars, poss, us))
    rms = float(np.sqrt(np.mean((tars - poss) ** 2)))

    fig, ax = plt.subplots(1, 2, figsize=(12, 4))
    ax[0].plot(ts, tars, 'k--', label='target')
    ax[0].plot(ts, poss, 'r', label='actual')
    ax[0].set_title(f'target vs actual  (RMS = {rms:.4f} rad)')
    ax[0].set_xlabel('t (s)'); ax[0].legend(); ax[0].grid(True)

    ax[1].plot(ts, us, 'b')
    ax[1].set_title('control output u')
    ax[1].set_xlabel('t (s)'); ax[1].set_ylabel('torque'); ax[1].grid(True)

    fig.tight_layout()
    fig.savefig(PNG, dpi=120)
    print(f"✅ 曲线：{PNG}")

    print(f"\n【PID 控制】RMS 误差 = {rms:.4f} rad   "
          f"({'✅ 达标' if rms < 0.01 else '❌ 超标'}，门槛 0.01)")
    print(f"     力矩范围 = [{us.min():+.2f}, {us.max():+.2f}] N·m  "
          f"(限幅 ±{U_LIM})")


if __name__ == "__main__":
    main()
