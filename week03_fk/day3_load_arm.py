# 1.导入库与基础设置
import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI, options="--width=1152 --height=700")  # 打开图形界面，并设置窗口大小
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0) # 以上三行是关掉界面上的多余的小黑框 让界面干净一点

# 2.设置物理环境
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)  # 设置重力，Z轴向下，大小为 9.81 m/s2

# 3.加载模型（地面+机械臂）
p.loadURDF("plane.urdf") # 加载一个无穷大的平面作为地板
arm = p.loadURDF("week03_fk/day3_two_link_arm_2.urdf", useFixedBase=True) # 读取写的urdf 并且把底座焊在地面上

# 4.检查关节信息
print("关节数:", p.getNumJoints(arm))  # getNumJoints：获取你模型里有几个能动的关节
for i in range(p.getNumJoints(arm)):
    info = p.getJointInfo(arm, i)
    print(f"  关节 {i}: {info[1].decode()}  type={info[2]}")

# 5.空转与调整相机
for _ in range(10):
    p.stepSimulation()

p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=50, cameraPitch=-25,
                             cameraTargetPosition=[0.5, 0, 0.5])

# 6.核心控制循环（让它动起来）
t = 0.0
try:
    while p.isConnected():
        # 控制关节0（joint1）
        p.setJointMotorControl2(arm, 0, p.POSITION_CONTROL,
                                targetPosition=0.8 * math.sin(t)) # 肩关节（0）目标角度是 0.8 * sin(t)，也就是在 ±0.8 弧度之间摆动
        # 控制关节1（joint2）
        p.setJointMotorControl2(arm, 1, p.POSITION_CONTROL,
                                targetPosition=-1.2 * math.sin(t + 1)) # 肘关节（1）目标角度是 -1.2 * sin(t + 1)
        p.stepSimulation()
        time.sleep(1/240)
        t += 1/240

# 7.退出保护
except KeyboardInterrupt:
    pass
p.disconnect()

# 界面操作：ctrl+左键：旋转视角    滚轮：缩放   ctrl+滚轮：平移