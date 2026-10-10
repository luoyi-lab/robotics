#!/usr/bin/env python3
"""PyBullet 实时窗口 —— 可用版本。

要点：
  1. 只关那三块预览面板，绝不要用 COV_ENABLE_GUI=0（会把 3D 画面一起关掉）
  2. 窗口尺寸必须小于屏幕分辨率，否则布局错乱、文字被切
     用户主屏 1536x864（1920x1080 @125% 缩放）→ 这里用 1152x700
  3. 相机在跑了几步之后再设置

操作：左键拖动 = 转视角；滚轮 = 缩放；Ctrl+左键拖动 = 平移。
"""
import math
import sys
import time

print("=" * 50)
print(f"Python: {sys.executable}")
print("=" * 50)

import pybullet as p
import pybullet_data

p.connect(p.GUI, options="--width=1152 --height=700")

p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
n = p.getNumJoints(robot)
print(f"机械臂已加载：{n} 个关节，id={robot}")

for _ in range(10):
    p.stepSimulation()

p.resetDebugVisualizerCamera(
    cameraDistance=2.5,
    cameraYaw=60,
    cameraPitch=-25,
    cameraTargetPosition=[0, 0, 0.5],
)

info = p.getDebugVisualizerCamera()
print(f"相机: dist={info[10]:.2f} pitch={info[8]:.2f} yaw={info[9]:.2f} target={info[11]}")

# 查一下机械臂末端在哪，确认它确实在原点附近
end = p.getLinkState(robot, n - 1)
print(f"末端位置: x={end[4][0]:.3f} y={end[4][1]:.3f} z={end[4][2]:.3f}")
print(f"底座位置: x={end[0][0]:.3f} y={end[0][1]:.3f} z={end[0][2]:.3f}")

print()
print(">>> 看窗口 <<<")
print("左键拖动转视角 / 滚轮缩放 / 关窗口或 Ctrl+C 结束")
print()

t = 0.0
try:
    while p.isConnected():
        for j in range(n):
            angle = 0.5 * math.sin(t * 1.2 + j * 0.6)
            p.setJointMotorControl2(
                robot, j, p.POSITION_CONTROL,
                targetPosition=angle, force=200,
            )
        p.stepSimulation()
        time.sleep(1 / 120)
        t += 1 / 120
except KeyboardInterrupt:
    print("\n收到 Ctrl+C")

if p.isConnected():
    p.disconnect()
print("=== 已退出 ===")
