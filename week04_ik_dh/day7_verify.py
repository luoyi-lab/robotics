# ============================================================
# W4 验证汇总（回归测试）
#   把 W4 学过的所有验证一次性跑完。
#   以后改任何东西，跑这一个脚本，就知道有没有把 W4 弄坏。
#
#   运行：  ~/pybullet-env/bin/python day7_verify.py
# ============================================================
import numpy as np

np.set_printoptions(precision=6, suppress=True)


# ============================================================
# 基础函数（从 W3/W4 各天汇总过来，所以这个脚本可以独立运行）
# ============================================================
def Rz(t):
    c, s = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s
    T[1, 0] = s; T[1, 1] = c
    return T


def Tz(d):
    T = np.eye(4); T[2, 3] = d; return T


def Tx(a):
    T = np.eye(4); T[0, 3] = a; return T


def Rx(al):
    c, s = np.cos(al), np.sin(al)
    T = np.eye(4)
    T[1, 1] = c; T[1, 2] = -s
    T[2, 1] = s; T[2, 2] = c
    return T


def dh(theta, d, a, alpha):
    """标准型 D-H"""
    return Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)


def dh_mod(theta, d, a, alpha):
    """改进型 D-H（Craig）"""
    return Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)


def dh_closed(theta, d, a, alpha):
    """课本闭式矩阵"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,         sa,       ca,      d],
        [0,          0,        0,      1],
    ])


def fk_2link(t1, t2, L1=1.0, L2=1.0):
    return (L1 * np.cos(t1) + L2 * np.cos(t1 + t2),
            L1 * np.sin(t1) + L2 * np.sin(t1 + t2))


def ik_2link(x, y, L1=1.0, L2=1.0):
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if c2 > 1.0 + 1e-9 or c2 < -1.0 - 1e-9:
        return []
    c2 = float(np.clip(c2, -1.0, 1.0))
    s2 = np.sqrt(1.0 - c2 * c2)
    out = []
    for sign in (+1.0, -1.0):
        ss = sign * s2
        out.append((np.arctan2(y, x) - np.arctan2(L2 * ss, L1 + L2 * c2),
                    np.arctan2(ss, c2)))
    return out


def jacobian(t1, t2, L1=1.0, L2=1.0):
    s1, c1 = np.sin(t1), np.cos(t1)
    s12, c12 = np.sin(t1 + t2), np.cos(t1 + t2)
    return np.array([[-L1 * s1 - L2 * s12, -L2 * s12],
                     [L1 * c1 + L2 * c12,  L2 * c12]])


def link_transform_w3(theta, L):
    """W3 Day 2 写的那个：Rz(θ)·Trans(L)"""
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta),  np.cos(theta), 0],
                  [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = R @ np.array([L, 0.0, 0.0])
    return T


# ============================================================
# 检查框架
# ============================================================
RESULTS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    RESULTS.append((name, ok))
    mark = "✅" if ok else "❌"
    print(f"  {mark} {name}")
    if detail:
        print(f"       {detail}")


rng = np.random.default_rng(0)

print("=" * 62)
print("W4 验证汇总")
print("=" * 62)

# ------------------------------------------------------------
# 1. IK 闭环：随机 500 点 × 2 解，FK 还原误差 < 1e-9
# ------------------------------------------------------------
print("\n[1] IK 闭环")
worst = 0.0
n_checked = 0
n_badlen = 0
for _ in range(500):
    t1, t2 = rng.uniform(-np.pi, np.pi, 2)
    px, py = fk_2link(t1, t2)
    sols = ik_2link(px, py)
    if len(sols) != 2:
        n_badlen += 1
        continue
    for a, b in sols:
        rx, ry = fk_2link(a, b)
        worst = max(worst, float(np.hypot(rx - px, ry - py)))
        n_checked += 1
check("500 个随机点，解数都是 2", n_badlen == 0, f"异常 {n_badlen} 个")
check("FK 还原误差 < 1e-9", worst < 1e-9,
      f"验证 {n_checked} 次，最大误差 = {worst:.3e}")

# ------------------------------------------------------------
# 2. 工作空间：洞里的点 []、太远的点 []、环上的点 2 解
# ------------------------------------------------------------
print("\n[2] 工作空间（L1=1.0, L2=0.5 → 内半径 0.5，外半径 1.5）")
hole = ik_2link(0.2, 0.0, 1.0, 0.5)     # r=0.2 < 0.5  洞内
far = ik_2link(1.8, 0.0, 1.0, 0.5)      # r=1.8 > 1.5  太远
ring_in = ik_2link(0.6, 0.0, 1.0, 0.5)  # 环上
ring_out = ik_2link(1.2, 0.0, 1.0, 0.5)  # 环上
check("洞内 (0.2,0) 返回 0 解", len(hole) == 0, f"实际 {len(hole)} 解")
check("太远 (1.8,0) 返回 0 解", len(far) == 0, f"实际 {len(far)} 解")
check("环上 (0.6,0) 和 (1.2,0) 各 2 解",
      len(ring_in) == 2 and len(ring_out) == 2,
      f"实际 {len(ring_in)} / {len(ring_out)} 解")

# ------------------------------------------------------------
# 3. det(J) = L1 · L2 · sin(θ2)
# ------------------------------------------------------------
print("\n[3] 雅可比行列式")
worst = 0.0
n = 0
for L1_, L2_ in [(1.0, 1.0), (1.0, 0.5), (1.5, 0.5)]:
    for _ in range(70):
        t1, t2 = rng.uniform(-np.pi, np.pi, 2)
        d = np.linalg.det(jacobian(t1, t2, L1_, L2_))
        worst = max(worst, abs(float(d) - L1_ * L2_ * np.sin(t2)))
        n += 1
check("det(J) == L1*L2*sin(θ2)，3 种臂长", worst < 1e-12,
      f"{n} 组，最大偏差 = {worst:.3e}")

# ------------------------------------------------------------
# 4. 奇异位形
# ------------------------------------------------------------
print("\n[4] 奇异位形")
px, py = fk_2link(0.3, 0.0)
sols0 = ik_2link(px, py)
same = (len(sols0) == 2 and np.allclose(sols0[0], sols0[1], atol=1e-9))
check("θ2 = 0 时两个解合并", same,
      f"解 = {sols0}" if sols0 else "无解")

orig_ok = all(np.hypot(*fk_2link(t1, np.pi)) < 1e-9
              for t1 in [0.0, 1.0, 2.5, -2.0])
check("θ2 = π 时任意 θ1 都到原点（零空间）", orig_ok,
      "4 个不同 θ1 全部落到 (0,0)")

# ------------------------------------------------------------
# 5. 完整 D-H：拆开乘 vs 闭式
# ------------------------------------------------------------
print("\n[5] 完整 D-H：四个初等矩阵连乘 vs 课本闭式")
worst = 0.0
for _ in range(200):
    th = rng.uniform(-np.pi, np.pi)
    d = rng.uniform(-1, 1)
    a = rng.uniform(-1, 1)
    al = rng.uniform(-np.pi, np.pi)
    worst = max(worst, float(np.max(np.abs(dh(th, d, a, al)
                                            - dh_closed(th, d, a, al)))))
check("拆开乘 ≡ 闭式矩阵", worst < 1e-12, f"200 组，最大偏差 = {worst:.3e}")

# ------------------------------------------------------------
# 6. D-H 令 d=0, α=0 → 必须等于 W3 的 link_transform
# ------------------------------------------------------------
print("\n[6] W3 的 link_transform 是完整 D-H 的特例")
worst = 0.0
for _ in range(200):
    th = rng.uniform(-np.pi, np.pi)
    L = rng.uniform(0.1, 2.0)
    worst = max(worst, float(np.max(np.abs(dh(th, 0.0, L, 0.0)
                                            - link_transform_w3(th, L)))))
check("dh(θ, 0, L, 0) ≡ W3 link_transform", worst < 1e-12,
      f"200 组，最大偏差 = {worst:.3e}")

# ------------------------------------------------------------
# 7. 标准型 vs 改进型：同一只臂，两张表
# ------------------------------------------------------------
print("\n[7] 标准型 vs 改进型")
L1x, L2x = 1.0, 1.0
worst = 0.0
for _ in range(200):
    t1, t2 = rng.uniform(-np.pi, np.pi, 2)
    std = (dh(t1, 0, L1x, np.pi / 2)
           @ dh(t2, 0, L2x, 0.0))[:3, 3]
    mod = (dh_mod(t1, 0, 0.0, 0.0)
           @ dh_mod(t2, 0, L1x, np.pi / 2)
           @ dh_mod(0, 0, L2x, 0.0))[:3, 3]
    worst = max(worst, float(np.max(np.abs(std - mod))))
check("标准型表 ≡ 改进型表（a/α 整列下移一格）", worst < 1e-12,
      f"200 组，最大偏差 = {worst:.3e}")

# ------------------------------------------------------------
# 8. 三连杆 α=+90° 的全零位
# ------------------------------------------------------------
print("\n[8] 三连杆 (d1=0.5, a2=1.0, a3=0.7, α1=+90°)")
p0 = (dh(0, 0.5, 0.0, np.pi / 2)
      @ dh(0, 0.0, 1.0, 0.0)
      @ dh(0, 0.0, 0.7, 0.0))[:3, 3]
check("全零位末端 = (1.7, 0, 0.5)",
      np.allclose(p0, [1.7, 0.0, 0.5], atol=1e-12),
      f"实际 = ({p0[0]:.4f}, {p0[1]:.4f}, {p0[2]:.4f})")

# ------------------------------------------------------------
# 汇总
# ------------------------------------------------------------
print("\n" + "=" * 62)
n_pass = sum(1 for _, ok in RESULTS if ok)
n_all = len(RESULTS)
if n_pass == n_all:
    print(f"全部通过 ✅   ({n_pass}/{n_all})")
else:
    print(f"有失败 ❌     ({n_pass}/{n_all})")
    for name, ok in RESULTS:
        if not ok:
            print(f"   失败项: {name}")
print("=" * 62)
