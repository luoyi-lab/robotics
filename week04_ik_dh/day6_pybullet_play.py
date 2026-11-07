import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import math

if __name__ == "__main__":
    # 1. 开启 GUI 模式
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 关掉重力（因为我们要用代码强制控制姿态）
    p.setGravity(0, 0, 0)
    
    # 调整一个好看的摄像机视角 (距离3米，俯仰角-30度，偏航45度)
    p.resetDebugVisualizerCamera(cameraDistance=3.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.8])

    # 2. 加载你的 URDF 机械臂
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "day6_arm3.urdf")
    
    print(f"尝试加载: {urdf_path}")
    try:
        arm = p.loadURDF(urdf_path, useFixedBase=True)
    except Exception as e:
        print(f"❌ 加载失败，请检查 URDF 路径是否正确: {e}")
        exit()

    print("🎬 动画开始！请在弹出的 3D 窗口中观看机械臂挥舞。")
    print("👉 想要退出，请直接点击右上角关闭窗口，或在终端按 Ctrl+C。")

    # 3. 开始持续的动画循环
    t = 0.0  # 时间变量，用来驱动正弦波
    try:
        # 只要窗口还开着，就一直循环
        while p.isConnected():
            
            # 用数学正弦/余弦函数生成丝滑的关节角度
            # 幅度可以根据需要调整：pi/2 是 90度
            t1 = np.sin(t * 0.5) * (np.pi / 2)         # 肩膀左右转动
            t2 = np.sin(t * 1.5) * (np.pi / 3) + 0.5   # 大臂上下俯仰 (加0.5是为了整体稍微抬高一点)
            t3 = np.cos(t * 1.0) * (np.pi / 2)         # 小臂上下弯曲

            # 强制设置关节角度 (因为是纯运动学动画，这样最稳定)
            p.resetJointState(arm, 0, t1)
            p.resetJointState(arm, 1, t2)
            p.resetJointState(arm, 2, t3)

            # 步进仿真引擎
            p.stepSimulation()
            
            # 控制动画帧率 (约 60 FPS)
            time.sleep(1./60.)
            
            # 时间流逝 (控制挥舞速度)
            t += 0.05 

    except KeyboardInterrupt:
        print("\n手动退出动画。")
    finally:
        if p.isConnected():
            p.disconnect()