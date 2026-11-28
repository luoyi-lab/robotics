import numpy as np
import matplotlib.pyplot as plt
import os
HERE = os.path.dirname(os.path.abspath(__file__))

EPS = 0.2

# 核心目标函数
def L(ratio, A, eps=EPS):
    return min(ratio * A, np.clip(ratio, 1 - eps, 1 + eps) * A)

# 终端打印表格
print(f"{'ratio':>7} | {'A=+1':>10} | {'A=-1':>10} | {'dL/dr(+)':>12} | {'dL/dr(-)':>12}")
print("-" * 62)
h = 1e-6
for r in [0.5, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.5]:
    # 数值梯度：(L(r+h) - L(r-h)) / 2h
    g1 = (L(r + h, +1.0) - L(r - h, +1.0)) / (2 * h)
    g2 = (L(r + h, -1.0) - L(r - h, -1.0)) / (2 * h)
    print(f"{r:>7.2f} | {L(r, +1.0):>10.4f} | {L(r, -1.0):>10.4f} | {g1:>12.4f} | {g2:>12.4f}")

# 画图
ratios = np.linspace(0.4, 1.6, 200)
L_pos = [L(r, +1.0) for r in ratios]
L_neg = [L(r, -1.0) for r in ratios]

plt.figure(figsize=(10, 5))
plt.plot(ratios, L_pos, label='A = +1 (Good Action)', color='green', linewidth=2)
plt.plot(ratios, L_neg, label='A = -1 (Bad Action)', color='red', linewidth=2)

# 画竖线
plt.axvline(0.8, color='gray', linestyle='--', label='clip bounds (0.8, 1.2)')
plt.axvline(1.2, color='gray', linestyle='--')

plt.title('PPO Clipped Objective Function')
plt.xlabel('ratio (new policy / old policy)')
plt.ylabel('L (Objective)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(HERE, "day6_ppo_curve.png"), dpi=150)