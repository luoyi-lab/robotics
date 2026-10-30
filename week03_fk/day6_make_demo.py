# 第一块 环境与依赖引入（搭台子）
import os   # 处理文件路径（找 URDF 和保存 GIF 用）
import math   # 数学库（算 sin 函数用）
import numpy as np    # 科学计算库（处理图像像素矩阵用）
import pybullet as p   # 物理仿真引擎本体
import pybullet_data   # 官方提供的模型数据包（里面包含 plane.urdf 地面模型）
from PIL import Image   # 图像处理库（把像素拼成图片，再把图片拼成 GIF）

# 第二块 路径与参数设置
HERE = os.path.dirname(os.path.abspath(__file__))   # HERE：获取你当前这个 Python 脚本所在的绝对路径。
URDF = os.path.join(HERE, "day3_two_link_arm_2.urdf")  
OUT  = os.path.join(HERE, "demo.gif")   # URDF / OUT：极其规范的做法。不管你把代码拷到哪，它都会在当前文件夹找文件 这能彻底避免“找不到文件”的报错。

W, H = 640, 480   # 分辨率
FPS = 25       # 帧数
N_FRAMES = 500          # 500 / 25 = 20 秒 出20秒的gif 

# 第三块 启动物理引擎与加载场景
p.connect(p.DIRECT)   # 无头模式 只在后台算物理和渲染 不弹出窗口
p.setAdditionalSearchPath(pybullet_data.getDataPath())   # 告诉引擎去哪找官方模型
p.setGravity(0, 0, -9.81)   # 设置重力加速度

p.loadURDF("plane.urdf")   # 加载无限大的地面 
arm = p.loadURDF(URDF, useFixedBase=True)   # 加载在URDF中写好的两连杆机械臂 并把底座焊丝

# 第四块 设置虚拟相机
# 相机：从斜上方看，target 放在臂的中间
view = p.computeViewMatrixFromYawPitchRoll(
    cameraTargetPosition=[1.0, 0, 0.5],
    distance=3.0, yaw=50, pitch=-25, roll=0, upAxisIndex=2,
)   # 把镜头放在距离目标 3 米的地方，向左偏 50 度，向下俯视 25 度，死死盯着坐标 [1.0, 0, 0.5]（也就是机械臂的中心）
proj = p.computeProjectionMatrixFOV(fov=60, aspect=W/H, nearVal=0.1, farVal=100)
# 设置相机的广角为 60 度，画面比例和分辨率一致，只能看到 0.1 米到 100 米范围内的东西

# 第五块 核心循环（演戏与截图）
frames = []   # 准备一个空列表装照片
for i in range(N_FRAMES):   # 循环500次
    t = i / FPS   # t是当前运行的物理时间
    # TODO: 设置两个关节角（用正弦，两个关节不同频率）
    # 使用 math.sin(频率 * 时间 + 相位)
    # 频率控制运动快慢，相位控制启动时的初始姿态
    t1 = 1.0 * math.sin(1.5 * t)             # 关节 1：振幅 1.0 弧度，角频率 1.5
    t2 = 1.2 * math.sin(3.0 * t + math.pi/2) # 关节 2：振幅 1.2 弧度，角频率 3.0（频率是关节1的两倍），加 90 度相位差
    p.resetJointState(arm, 0, t1)
    p.resetJointState(arm, 1, t2)
    # 根据当前时间，用正弦函数算出两个关节应该在哪个角度，然后强塞给机械臂

    p.stepSimulation()   # 引擎向前推进一帧，应用刚算出的关节角，更新所有物体的位置

    _, _, rgb, _, _ = p.getCameraImage(W, H, view, proj,
                                       renderer=p.ER_TINY_RENDERER)
    # 用刚刚设置好的镜头拍张照。rgb 拿到的是一张高度为 480，宽度为 640 的图像数据
    arr = np.array(rgb, dtype=np.uint8).reshape(H, W, 4)[:, :, :3]
    frames.append(Image.fromarray(arr))
    # PyBullet 拍出来的照片有 4 个通道（红绿蓝 + 透明度）但我们 GIF 不需要透明度，所以 [:, :, :3] 就是把最后一个 Alpha 通道砍掉，只留 RGB
    # 然后把它变成 PIL 库能认识的 Image 对象，塞进 frames 列表里。

    if (i + 1) % 100 == 0:
        print(f"  已渲染 {i+1}/{N_FRAMES}")
    # 进度条。每拍 100 张告诉你一声，防止你以为程序卡死了

p.disconnect()   # 断开物理引擎，释放内存

frames[0].save(OUT, save_all=True, append_images=frames[1:],
               duration=int(1000 / FPS), loop=0)
# 合成 GIF：用第一张图作为门面，后面 499 张紧跟着。
# duration=40 毫秒（1000/25）代表每张图停留 40ms。loop=0 代表无限循环播放
print(f"已保存 {OUT}  ({os.path.getsize(OUT)//1024} KB)")
# 告诉你文件存在哪了，文件有多大