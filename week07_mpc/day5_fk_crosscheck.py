import numpy as np
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "fk_cases.csv")

rng = np.random.default_rng(20261127)   # ← 固定种子，保证可复现

def fk_2link_geom(t1, t2, L1, L2):
    """W3 的几何法：直接三角函数"""
    x = L1 * np.cos(t1) + L2 * np.cos(t1 + t2)
    y = L1 * np.sin(t1) + L2 * np.sin(t1 + t2)
    return x, y

N = 50
rows = []
for _ in range(N):
    t1 = rng.uniform(-np.pi, np.pi)
    t2 = rng.uniform(-np.pi, np.pi)
    L1 = rng.uniform(0.2, 2.0)
    L2 = rng.uniform(0.2, 2.0)
    x, y = fk_2link_geom(t1, t2, L1, L2)
    rows.append((t1, t2, L1, L2, x, y))

with open(CSV, "w") as f:
    f.write("theta1,theta2,L1,L2,x,y\n")
    for r in rows:
        f.write("%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n" % r)

print(f"写了 {N} 组到 {CSV}")