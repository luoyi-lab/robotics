import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import time
import os

# 核心函数 解一拍的MPC 返回第一步的控制量
def mpc_step(p, v, p_ref, N, Q=1.0, R=0.1, u_lim=2.0, dt=0.1):
    a = cp.Variable(N)   # 决策变量：未来 N 步的加速度

    # 用列表递推预测未来
    ps, vs = [p], [v]
    for k in range(N):
        ps.append(ps[k] + dt * vs[k] + (dt**2 / 2) * a[k])
        vs.append(vs[k] + dt * a[k])

    # 代价函数：位置误差 + 控制量惩罚
    cost = sum(Q * cp.square(ps[k] - p_ref) for k in range(1, N + 1))    # 决定要“逼近目标 这也是p最后逼近目标的原因
    cost += R * cp.sum_squares(a)
    # 上两行代码就是在告诉系统：“我的要求是：让预测位置 ps[k] 离目标 p_ref 越近越好（误差平方最小），同时油门 a 踩得越小越好

    # 物理约束
    cons = [a >= -u_lim, a <= u_lim]   # 电机输出的加速度有上限

    # 定义问题与求解
    prob = cp.Problem(cp.Minimize(cost), cons)
    prob.solve()

    # 安全检查
    if prob.status != 'optimal':
        print(f"第 {k} 拍求解失败: {prob.status}")
        return 0.0   # 如果解不出来 默认不踩油门

    # 关键灵魂 只返回第一步的加速度
    return float(a.value[0])

# 2.闭环主循环
def run_closed_loop(N=10, steps=120, p_ref=10.0, u_lim=2.0, change_target_step=None):
    p, v = 0.0, 0.0
    hist_p, hist_v, hist_a = [], [], []
    dt = 0.1
    change_target_value=0.0

    start_time = time.time()

    for k in range(steps):
        # 实验D 中途换目标
        if change_target_step is not None and k >= change_target_step:
            current_pref = change_target_value
        else:
            current_pref = p_ref

        # 1.解算第一步
        a0 = mpc_step(p, v, current_pref, N, u_lim=u_lim)

        hist_p.append(p)
        hist_v.append(v)
        hist_a.append(a0)

        # 2.真实物理系统往前走一步（注意：是用 a0 更新真实的 p, v）
        p = p + dt * v + (dt**2 / 2) * a0
        v = v + dt * a0
        
    elapsed = time.time() - start_time   # 计算整个 for 循环跑了多少秒
    return np.array(hist_p), np.array(hist_v), np.array(hist_a), elapsed   # 把三个列表转成 NumPy 数组

# 3.执行四个实验
if __name__ == "__main__":
    HERE = os.path.dirname(os.path.abspath(__file__))
    dt = 0.1
    steps = 120
    t_arr = np.arange(steps) * dt

    # 实验A 基准闭环
    print("=== 实验 A: 基准闭环 ===")
    p_a, v_a, a_a, t_a = run_closed_loop(N=10) # 调用函数
    # run_closed_loop 返回 4 个变量：真实位置轨迹 p_a、速度轨迹 v_a、实际输出的加速度序列 a_a、以及整个循环跑完花了多少时间 t_a
    print(f"末位置: {p_a[-1]:.4f}, 最大加速度: {np.max(np.abs(a_a)):.4f}, 耗时: {t_a:.2f}s")
    # p_a[-1] 是最后一次循环停在的位置
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(t_arr, p_a, label='Position')
    axs[0].axhline(10, color='k', linestyle='--', label='Target')
    axs[0].set_ylabel('Position')
    axs[1].plot(t_arr, v_a, label='Velocity', color='orange')
    axs[1].set_ylabel('Velocity')
    axs[2].plot(t_arr, a_a, label='Acceleration (a0)', color='green')
    axs[2].axhline(2.0, color='r', linestyle=':', label='u_lim')
    axs[2].axhline(-2.0, color='r', linestyle=':')
    axs[2].set_ylabel('Accel')
    axs[2].set_xlabel('Time (s)')
    plt.suptitle("Experiment A: Baseline Closed Loop (N=10)")
    plt.tight_layout()
    plt.savefig(os.path.join(HERE, "day3_mpc_closed_loop.png"), dpi=150)
    plt.close()

    # ================= 实验 B：预测时域 N 的影响 =================
    print("\n=== 实验 B: 预测时域 N 的影响 ===")
    N_list = [5, 10, 20]
    plt.figure(figsize=(10, 5))
    for n in N_list:
        p_hist, _, _, t_elapsed = run_closed_loop(N=n)
        print(f"N={n:2d} | 末位置: {p_hist[-1]:.4f} | 超调: {np.max(p_hist):.4f} | 耗时: {t_elapsed:.2f}s")
        plt.plot(t_arr, p_hist, label=f'N={n} (time: {t_elapsed:.2f}s)')
        
    plt.axhline(10, color='k', linestyle='--', label='Target')
    plt.title("Experiment B: Effect of Prediction Horizon N")
    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(HERE, "day3_horizon_compare.png"), dpi=150)
    plt.close()

    # ================= 实验 C：有约束 vs 无约束 =================
    print("\n=== 实验 C: 有约束 vs 无约束 ===")
    _, _, a_constrained, _ = run_closed_loop(N=10, u_lim=2.0)
    _, _, a_unconstrained, _ = run_closed_loop(N=10, u_lim=100.0) # 放开约束
    print(f"有约束 max|a|: {np.max(np.abs(a_constrained)):.4f}")
    print(f"无约束 max|a|: {np.max(np.abs(a_unconstrained)):.4f} (物理不可达)")

    # ================= 实验 D：中途换目标 =================
    print("\n=== 实验 D: 中途换目标 ===")
    p_d, _, _ , _= run_closed_loop(N=10, steps=220, change_target_step=70)
    print(f"末位置: {p_d[-1]:.4f}")
    
    plt.figure(figsize=(10, 5))
    t_arr_d = np.arange(len(p_d)) * dt
    plt.plot(t_arr_d, p_d, label='Position')
    plt.axvline(70*dt, color='r', linestyle='--', label='Target Changed to 0')
    plt.title("Experiment D: Changing Target Mid-Run")
    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(HERE, "day3_change_target.png"), dpi=150)
    plt.close()

    print("\n✅ 所有实验完成，图片已保存至:", HERE)