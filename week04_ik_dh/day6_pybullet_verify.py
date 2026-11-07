import pybullet as p
import pybullet_data
import numpy as np
import os  # 新增：用来处理文件路径

# ================= 你的 D-H 基础函数 =================
def Rz(t):
    T = np.eye(4); c, s = np.cos(t), np.sin(t)
    T[0,0]=c; T[0,1]=-s; T[1,0]=s; T[1,1]=c; return T
def Tz(d):
    T = np.eye(4); T[2,3]=d; return T
def Tx(a):
    T = np.eye(4); T[0,3]=a; return T
def Rx(al):
    T = np.eye(4); c, s = np.cos(al), np.sin(al)
    T[1,1]=c; T[1,2]=-s; T[2,1]=s; T[2,2]=c; return T

def dh(theta, d, a, alpha):
    return Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)

def fk_dh(t1, t2, t3):
    T = (dh(t1, 0.5, 0.0, np.pi/2) @
         dh(t2, 0.0, 1.0, 0.0) @
         dh(t3, 0.0, 0.7, 0.0))
    return T[:3, 3]

# ================= PyBullet 对接测试 =================
if __name__ == "__main__":
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 🌟 核心修改点：获取当前 .py 脚本所在的绝对路径，拼出 URDF 的绝对路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "day6_arm3.urdf")
    
    print(f"尝试加载 URDF: {urdf_path}")
    
    # 加载绝对路径
    arm = p.loadURDF(urdf_path, useFixedBase=True)

    print("开始与 PyBullet 对账 10 组随机数据...")
    np.random.seed(42)

    for i in range(10):
        t1 = np.random.uniform(-np.pi, np.pi)
        t2 = np.random.uniform(-np.pi/2, np.pi/2)
        t3 = np.random.uniform(-np.pi/2, np.pi/2)

        p.resetJointState(arm, 0, t1)
        p.resetJointState(arm, 1, t2)
        p.resetJointState(arm, 2, t3)

        sim_state = p.getLinkState(arm, 3, computeForwardKinematics=True)
        sim_pos = np.array(sim_state[4]) 

        mine_pos = fk_dh(t1, t2, t3)

        try:
            np.testing.assert_allclose(sim_pos, mine_pos, atol=1e-6)
            print(f"✅ 第 {i+1:02d} 组通过 | 误差: {np.linalg.norm(sim_pos - mine_pos):.2e}")
        except AssertionError as e:
            print(f"❌ 第 {i+1:02d} 组失败！")
            print(f"   仿真值: {sim_pos}")
            print(f"   理论值: {mine_pos}")
            break

    p.disconnect()