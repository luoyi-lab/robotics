#!/usr/bin/env python3
"""PyBullet 交互式测试：窗口会一直开着，机械臂持续摆动。

关闭方式：直接关掉窗口，或在终端按 Ctrl+C。
"""
import math
import time

import pybullet as p
import pybullet_data

print("=== 连接仿真器 ===")
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

print("=== 加载模型 ===")
p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
n = p.getNumJoints(robot)
print(f"机械臂已加载，{n} 个关节")

# 调一下视角，看得清楚点
p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=50,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.5],
)

print()
print(">>> 窗口已打开 <<<")
print("机械臂会持续摆动。想结束：关掉窗口，或在终端按 Ctrl+C")
print()

t = 0.0
try:
    while p.isConnected():
        # 7 个关节各按不同相位摆动，看起来像在"呼吸"
        for j in range(n):
            angle = 0.6 * math.sin(t * 1.5 + j * 0.6)
            p.setJointMotorControl2(
                robot, j, p.POSITION_CONTROL,
                targetPosition=angle, force=200,
            )
        p.stepSimulation()
        time.sleep(1 / 240)
        t += 1 / 240
except KeyboardInterrupt:
    print("\n收到 Ctrl+C")

if p.isConnected():
    p.disconnect()
print("=== 已退出 ===")
