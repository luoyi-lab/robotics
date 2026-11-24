# 代码训练 涵盖了从最基础的不等式、盒子约束，到 MPC 雏形的核心逻辑
import cvxpy as cp

def run_exercise_1():
    print("\n--- 练习1：带不等式约束")
    x = cp.Variable()
    y = cp.Variable()
    objective = cp.Minimize(cp.square(x - 3) + cp.square(y - 1))
    constraints = [x + y <= 2]

    prob = cp.Problem(objective, constraints)
    prob.solve()

    print(f"status = {prob.status}")
    if prob.status == 'optimal':   # 只有当状态是 'optimal'（成功找到解）时，才允许执行下面的代码
        print(f"x={x.value:.8f}, y = {y.value:.8f}")
        print(f"最优值 = {prob.value:.8f}")

def run_exercise_2():
    print("\n--- 练习 2: box 约束 ---")
    x = cp.Variable()
    y = cp.Variable()
    objective = cp.Minimize(cp.square(x - 3) + cp.square(y - 1))
    constraints = [
        x >= -1, x <= 1,
        y >= -1, y <= 1
    ]
    
    prob = cp.Problem(objective, constraints)
    prob.solve()
    
    print(f"status = {prob.status}")
    if prob.status == 'optimal':
        print(f"x = {x.value:.8f}, y = {y.value:.8f}")
        print(f"最优值 = {prob.value:.8f}")

def run_exercise_3():
    print("\n--- 练习 3: 不可行问题 ---")
    x = cp.Variable()
    objective = cp.Minimize(x)
    constraints = [x >= 1, x <= 0]
    
    prob = cp.Problem(objective, constraints)
    prob.solve()
    
    print(f"status = {prob.status}")
    print(f"x.value = {x.value}") # 期望是 None

def run_exercise_4(R_weight = 0.1):
    print(f"\n--- 练习 4: MPC 雏形 (R = {R_weight}) ---")
    # Day 1 模型：dt=0.1, p0=0, v0=0, 一步预测 p1 = 0.005 * a0
    dt = 0.1
    a0 = cp.Variable()
    # p1 = 0.005 * a0   # p[k+1] = p[k] + dt·v[k] + (dt²/2)·a[k]  计算得到的
    coef = dt**2 / 2.0  # 这样可以实时更新p1
    p1 = coef * a0

    # 目标：逼近 10，惩罚控制量
    objective = cp.Minimize(cp.square(p1 - 10) + R_weight * cp.square(a0))
    # cp.square(p1 - 10)：位置误差的平方   R_weight * cp.square(a0)：踩油门的代价（为了省电，不希望踩太大）
    # 目标是让这两者加起来最小

    prob = cp.Problem(objective)
    prob.solve()

    print(f"status = {prob.status}")
    if prob.status == 'optimal':
        print(f"a0 = {a0.value:.8f}")
        print(f"p1 = {p1.value:.8f}")
        print(f"最优值 = {prob.value:.8f}")

def run_exercise_5():
    print("\n--- 练习 5: 同一问题，加执行器饱和 ---")
    dt = 0.1
    a0 = cp.Variable()
    coef = dt**2 / 2.0
    p1 = coef * a0
    objective = cp.Minimize(cp.square(p1 - 10) + 0.1 * cp.square(a0))

    # 添加约束 a0在 [-0.2, 0.2] 之间
    constraints = [a0 >= -0.2, a0 <= 0.2]

    prob = cp.Problem(objective, constraints)
    prob.solve()
    
    print(f"status = {prob.status}")
    if prob.status == 'optimal':
        print(f"a0 = {a0.value:.8f} (被卡在边界)")
        print(f"p1 = {p1.value:.8f}")
        print(f"最优值 = {prob.value:.8f}")

if __name__ == "__main__":   # 如果当前这个代码文件是被当做‘主程序’直接运行的，那就执行下面的代码
    run_exercise_1()
    run_exercise_2()
    run_exercise_3()
    run_exercise_4(R_weight=0.1)
    run_exercise_4(R_weight=0.0001) # 验证 R 变小后的激进表现
    run_exercise_5()