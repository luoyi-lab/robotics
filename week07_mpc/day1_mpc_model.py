# 完整代码训练
import numpy as np
import matplotlib.pyplot as plt
import os

# 基础设置
HERE = os.path.dirname(os.path.abspath(__file__))   # ← 两边各两个下划线
dt = 0.1
A = np.array([[1.0, dt], [0.0, 1.0]])
B = np.array([dt**2 / 2.0, dt])  # ⚠️ 必须是 (2,) 的形状

# 任务A 开环物理引擎

def predict(x0, U):
    """
    给定初始状态 x0=[p,v] 和一个长度 N 的控制序列 U，
    返回从 k=0 到 k=N 的完整轨迹，形状 (N+1, 2)
    """
    N = len(U)   # 要求传入的 U 是一个列表
    X = np.zeros((N + 1, 2))
    X[0] = x0

    for k in range(N):
        # 核心公式：x[k+1] = A * x[k] + B * a[k]
        X[k+1] = A @ X[k] + B * U[k]
        
    return X

# 任务B & C 三条轨迹对比与打印

if __name__ == "__main__":
    N = 10

    # 三种控制序列
    U_accel = [1.0] * N   # 它等价于 [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] 
                          # 这代表一个“持续加速”的控制序列。在这 1 秒（10步）里，你每一秒都死死踩住油门，给小车输入 a = 1.0 的加速度
    U_uniform = [0.0] * N
    U_brake = [-1.0] * N

    # 三种初始状态
    x0_accel = [0.0, 0.0]    # 静止开始
    x0_uniform = [0.0, 1.0]  # 初速 1
    x0_brake = [0.0, 1.0]    # 初速 1

    # 跑预测
    X_accel = predict(x0_accel, U_accel)
    X_uniform = predict(x0_uniform, U_uniform)
    X_brake = predict(x0_brake, U_brake)

    # 1. 打印对照表 (任务 C)
    print(f"{'输入':<15} | {'末位置 p':<10} | {'末速度 v':<10}")
    print("-" * 45)
    print(f"{'a = +1 (加速)':<12} | {X_accel[-1, 0]:<10.4f} | {X_accel[-1, 1]:<10.4f}")
    print(f"{'a = 0  (匀速)':<12} | {X_uniform[-1, 0]:<10.4f} | {X_uniform[-1, 1]:<10.4f}")
    print(f"{'a = -1 (刹车)':<12} | {X_brake[-1, 0]:<10.4f} | {X_brake[-1, 1]:<10.4f}")

    # 2. 画图 (任务 B)
    # 生成时间轴
    t = np.arange(N + 1) * dt
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    # 左图：位置 p 随时间的变化
    ax1.plot(t, X_accel[:, 0], 'o-', label='a = +1 (Accel)')
    ax1.plot(t, X_uniform[:, 0], 's-', label='a = 0 (Uniform)')
    ax1.plot(t, X_brake[:, 0], '^-', label='a = -1 (Brake)')
    ax1.set_title('Position over Time')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position p')
    ax1.legend()
    ax1.grid(True)
    
    # 右图：速度 v 随时间的变化
    ax2.plot(t, X_accel[:, 1], 'o-', label='a = +1 (Accel)')
    ax2.plot(t, X_uniform[:, 1], 's-', label='a = 0 (Uniform)')
    ax2.plot(t, X_brake[:, 1], '^-', label='a = -1 (Brake)')
    ax2.set_title('Velocity over Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity v')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    # 保存图片（因为 WSL 弹不出窗口）
    save_path = os.path.join(HERE, 'day1_trajectories.png')
    plt.savefig(save_path, dpi=150)
    print(f"\n✅ 图片已保存至: {save_path}")