# 标准型vs改进型  Standard D-H vs Modified D-H
# 核心区别：
# 标准型：把坐标系建在连杆的末端（也就是下一个关节的轴上）。所以它的 Zi轴对应的是第 i+1个关节的转轴
# 改进型：把坐标系建在连杆的起点（也就是当前关节的轴上）。所以它的 Zi轴对应的是第 i 个关节的转轴

# 标准型的表：

# i	 θ	 d	 a	 α
# 1	 θ1	 0	 L1	 +90°
# 2	 θ2	 0	 L2	 0

# 换成改进型的表，同一只臂：

# i	     θ	  d	  a	    α
# 1      θ1	  0	  0	    0 
# 2	     θ2	  0	  L1   +90° 
# 工具	 —	  0	  L2	0 
# 因为改进型Zi轴对应的是第 i 个关节的转轴 所以最后还没有到末端 需要一个工具变换 才能到达终点

# 标准型：dh_std(t1,0,L1,π/2) @ dh_std(t2,0,L2,0)    
# 改进型：dh_mod(t1,0,0,0) @ dh_mod(t2,0,L1,π/2) @ dh_mod(0,0,L2,0)

# 关键点：数学矩阵的连乘顺序不一样
# 标准型：动作顺序：绕Z转 → 沿Z平移 → 沿X平移 → 绕X转  Rz(θ) @ Tz(d) @ Tx(a) @ Rx(α)
# 改进型：动作顺序：绕X转 → 沿X平移 → 绕Z转 → 沿Z平移  Rx(α) @ Tx(a) @ Rz(θ) @ Tz(d)

# 代码训练：
import numpy as np

# 定义基础工具 四个矩阵
def Rz(t):
    c, s = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s
    T[1, 0] = s; T[1, 1] = c
    return T

def Tz(d):
    T = np.eye(4)
    T[2, 3] = d
    return T

def Tx(a):
    T = np.eye(4)
    T[0, 3] = a
    return T

def Rx(al):
    c, s = np.cos(al), np.sin(al)
    T = np.eye(4)
    T[1, 1] = c; T[1, 2] = -s
    T[2, 1] = s; T[2, 2] =  c
    return T

# 定义标准型与改进型D-H变换
def dh_std(theta, d, a, alpha):
    return Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)

def dh_mod(theta, d, a, alpha):
    return Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)

# 正运动学求解
L1, L2 = 1.0, 1.0

def fk_std(t1, t2):  # 标准型连乘
    T1 = dh_std(t1, 0, L1, np.pi/2) 
    T2 = dh_std(t2, 0, L2, 0)
    T = T1 @ T2
    return T[:3, 3]  # 提取xyz

def fk_mod(t1, t2):  # 改进型连乘
    T1 = dh_mod(t1, 0, 0, 0)
    T2 = dh_mod(t2, 0, L1, np.pi/2)
    T3 = dh_mod(0, 0, L2, 0)  # 工具变换
    T = T1 @ T2 @ T3
    return T[:3, 3]

# 跑2000组随机角度验证
if __name__ == "__main__":    # 如果直接运行这个脚本，才执行下面的代码
    print("开始执行 2000 组随机角度对比验证...")
    np.random.seed(42) 
    
    max_error = 0.0
    all_passed = True
    
    for i in range(2000):
        t1_rand = np.random.uniform(-np.pi, np.pi)
        t2_rand = np.random.uniform(-np.pi, np.pi)
        
        pos_std = fk_std(t1_rand, t2_rand)
        pos_mod = fk_mod(t1_rand, t2_rand)
        
        if not np.allclose(pos_std, pos_mod, atol=1e-12):
            print(f"❌ 发现不匹配！第 {i+1} 组测试失败！")
            print(f"标准型: {pos_std}, 改进型: {pos_mod}")
            all_passed = False
            break
            
        err = np.linalg.norm(pos_std - pos_mod)
        max_error = max(max_error, err)
        
    if all_passed:
        print("\n✅ 完美！2000 组测试全部通过！")
        print(f"两种方法计算出的最大误差为: {max_error:.3e}")