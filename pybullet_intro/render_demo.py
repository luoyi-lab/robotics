#!/usr/bin/env python3
"""PyBullet 无界面渲染：不开窗口，直接渲染成 GIF 动图。

用纯 CPU 渲染器（TinyRenderer），不需要显卡、不需要显示器，稳定可靠。
"""
import math
import os

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

OUT = os.path.dirname(os.path.abspath(__file__))
W, H = 640, 480
FPS = 30
N_FRAMES = 120          # 120 帧 / 30fps = 4 秒动画

print("=== 1) 连接仿真器（无界面 DIRECT 模式）===")
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

print("=== 2) 加载模型 ===")
p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
n = p.getNumJoints(robot)
print(f"  机械臂 {n} 个关节")

# 相机：从斜上方看向机械臂
view = p.computeViewMatrixFromYawPitchRoll(
    cameraTargetPosition=[0, 0, 0.6],
    distance=2.0, yaw=50, pitch=-20, roll=0, upAxisIndex=2,
)
proj = p.computeProjectionMatrixFOV(
    fov=60, aspect=W / H, nearVal=0.1, farVal=100,
)

print(f"=== 3) 渲染 {N_FRAMES} 帧 ===")
frames = []
for i in range(N_FRAMES):
    t = i / FPS
    # 关节按不同相位摆动
    for j in range(n):
        angle = 0.6 * math.sin(t * 1.5 + j * 0.6)
        p.setJointMotorControl2(
            robot, j, p.POSITION_CONTROL, targetPosition=angle, force=200,
        )
    # 每帧之间推进物理
    for _ in range(4):
        p.stepSimulation()

    _, _, rgb, _, _ = p.getCameraImage(
        W, H, view, proj, renderer=p.ER_TINY_RENDERER,
    )
    arr = np.array(rgb, dtype=np.uint8).reshape(H, W, 4)[:, :, :3]
    frames.append(Image.fromarray(arr))

    if (i + 1) % 30 == 0:
        print(f"  已渲染 {i + 1}/{N_FRAMES} 帧")

p.disconnect()

print("=== 4) 保存文件 ===")
gif_path = os.path.join(OUT, "demo.gif")
frames[0].save(
    gif_path, save_all=True, append_images=frames[1:],
    duration=int(1000 / FPS), loop=0,
)
print(f"  {gif_path}  ({os.path.getsize(gif_path) // 1024} KB)")

png_path = os.path.join(OUT, "demo_frame.png")
frames[N_FRAMES // 2].save(png_path)
print(f"  {png_path}")

print("=== 完成 ===")
