# 解出θ1、θ2 
# 第一步 先算θ2  利用余弦定理
# cos θ2 = (x² + y² - L1² - L2²) / (2·L1·L2)  ≡  c2
# 因为 cosθ2的值域只能在 [−1,1]之间。如果你算出来的 c2 大于 1 或小于 -1，说明目标点够不着

# 第二步 从 c2 拿两个 θ2
# s2 = ±√(1 - c2²)         ← 正负号 = 两个解 = 肘上 / 肘下
# θ2 = atan2(s2, c2)   反三角函数
# arccos(c2) 算出来的角度只能在 [0,π]（即 0~180度）。丢失如果手臂需要往反方向弯（负数）的情况
# atan2(y, x) 是编程语言里的万能反三角函数。它同时知道 y（正弦值）和 x（余弦值）的符号，能判断出角度在第一、第二、第三还是第四象限，返回值是完整的 
# (−π,π]范围

# 第三步 再算θ1
# 1.正运动学FK展开：
# x = L1·cos(θ1) + L2·cos(θ1+θ2)
# y = L1·sin(θ1) + L2·sin(θ1+θ2)
# 2.三角公式展开：
# 用和角公式把 cos(θ1+θ2) 拆成 cosθ1·cosθ2 - sinθ1·sinθ2，把 sin(θ1+θ2) 拆成 sinθ1·cosθ2 + cosθ1·sinθ2
# 3.换元
# 令 k1 = L1 + L2·c2，k2 = L2·s2
# 公式变为：
# x = k1·cosθ1 - k2·sinθ1
# y = k1·sinθ1 + k2·cosθ1
# 4.上两行本质为旋转运动
# 这在数学上等于把一个向量 (k1,k2)旋转了 θ1 的角度
# atan2(y, x) 是求解手相对于原点的总角度（设为 α）
# atan2(k2, k1) 是求解内部构成的那个角度（设为ϕ）  
# 所以 θ1就等于总角度减去内部角度：θ1 = atan2(y, x) - atan2(k2, k1)   （画个图很好理解 一开始有个k1k2点  角度为ϕ 转了θ1   总角度变为α   θ1=α-ϕ）

# 整理成最终要写的三步：
# c2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)
# s2 = ±sqrt(1 - c2*c2)
# θ2 = atan2(s2, c2)
# θ1 = atan2(y, x) - atan2(L2*s2, L1 + L2*c2)

# 代码训练：
# 1.ik求解
import numpy as np

def ik_2link(x, y, L1=1.0, L2=1.0):
    """给目标点 (x, y)，返回所有解的列表。
    无解 → []
    一般 → 两个解 [(t1a, t2a), (t1b, t2b)]
    """
    # --- 第一步：算 c2 ---
    c2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)

    # --- 第二步：够不着就返回空 ---
    if c2 > 1.0 + 1e-9 or c2 < -1.0 - 1e-9:   # 有浮动差距
        return []

    c2 = float(np.clip(c2, -1.0, 1.0)) 
    # --- 第三步：两个解 ---
    s2 = np.sqrt(1.0 - c2 * c2)   # np.sqrt求平方根 这里求的是s2的绝对值
    solutions = []
    for sign in (+1.0, -1.0):   # 循环遍历正负号
        ss = sign * s2   # 得到带有正负号的实际正弦值
        t2 = np.arctan2(ss, c2)    # 计算第二个关节的角度θ2
        t1 = np.arctan2(y, x) - np.arctan2(L2 * ss, L1 + L2 * c2)   # 计算第二个关节的角度θ1
        solutions.append((t1, t2))

    return solutions


def fk_2link(t1, t2, L1=1.0, L2=1.0):   # 正运动学验证
    """W3 写过的，两行公式"""
    x = L1 * np.cos(t1) + L2 * np.cos(t1 + t2)
    y = L1 * np.sin(t1) + L2 * np.sin(t1 + t2)
    return x, y

# 2.FK-IK 闭环验证
# 随机生成 100 个关节角，用 FK 算出一个手的目标点；
# 然后把这个目标点喂给 IK，看 IK 能不能吐出两个解，并且这两个解再用 FK 算回去，一定要和原来的目标点一模一样
# 随机生成 100 个目标点
N = 100
ok_t1 = np.random.uniform(-np.pi, np.pi, N)   # θ1生成100组-180°到180°之间的随机关节角度
ok_t2 = np.random.uniform(-np.pi, np.pi, N)   # θ2生成100组-180°到180°之间的随机关节角度

count = 0   # 计数器归零。每成功验证一组解，它就加 1，用来最后打印成绩单
for i in range(N):
    # 1. 先用随机角度跑 FK 得到目标点 (px, py)
    px, py = fk_2link(ok_t1[i], ok_t2[i])   # 把刚才随机的第 i 个角度拿出来，丢进正运动学公式里，算出机器手在空间的绝对坐标px py
    
    # 2. 把目标点喂给 IK
    sols = ik_2link(px, py)   # 给 (px, py)，算关节角是多少  sols 会返回一个列表，里面装着算出来的解
    
    # 3. 断言：一般位置应该有两个解
    assert len(sols) == 2, "一般位置应该有两个解"
    
    # 4. 遍历 IK 算出来的两个解
    for (t1, t2) in sols:   # sols是一个包含元组的列表，比如：[ (0.1, 0.2), (0.3, 0.4) ]
        # 5. 把这组解再跑一次 FK，算回坐标 (rx, ry)
        rx, ry = fk_2link(t1, t2)
        
        # 6. 断言：算回来的坐标必须和目标点一样！
        # 必须用 np.allclose，不能用 ==，因为浮点数有精度误差
        assert np.allclose([rx, ry], [px, py], atol=1e-9), f"解不对: {(rx,ry)} 不等于 {(px,py)}"
        count += 1

print(f"验证 {count} 次，全部通过")

# (1)验证目标点在“内圈”
sols_inner = ik_2link(0.5, 0)   # 调用逆运动学函数 将点设为(0.5, 0)
# 如果真的够不着，应该返回 []。如果够得着（默认L1=L2=1就是够得着的），应该返回 2 个解。
# 我们这里写一个通用的断言：不管返回什么，如果有解，必须能FK还原！
if len(sols_inner) > 0:   # 证明IK算出了解
    for t1, t2 in sols_inner:
        rx, ry = fk_2link(t1, t2)
        assert np.allclose([rx, ry], [0.5, 0]), "内圈点IK反推后，FK无法还原！"   # 如果这段断言不对程序崩溃 会打印蓝色的字
    print("测试 1 (内圈点): 可达，且 IK-FK 闭环验证通过。")
else:
    print("测试 1 (内圈点): 判定为不可达，返回 []。")

# (2)验证目标点“太远”
sols_far = ik_2link(3.0, 0)
# 断言：必须返回空列表
assert len(sols_far) == 0, "太远的点竟然算出了解，代码有漏洞！"
print("测试 2 (太远): 成功拦截，返回 []。")

# (3)验证负方向点
sols_neg = ik_2link(-0.5, 0)
# 断言：必须能解出来，且有两个解
assert len(sols_neg) == 2, "负方向点应该有两个解"

for t1, t2 in sols_neg:
    rx, ry = fk_2link(t1, t2)
    # 核心！验证算回来的 x 坐标必须是 -0.5
    assert np.allclose([rx, ry], [-0.5, 0]), f"负方向点还原失败！算出来是 {(rx, ry)}"
print("测试 3 (负方向): 成功解出，且 atan2 象限处理完美，FK 还原通过。")