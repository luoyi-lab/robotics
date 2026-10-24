import numpy as np
def make_transform(R = None, p = None):
    T = np.eye(4)
    if R is not None:
            T[:3, :3] = R   
    if p is not None:
            T[:3, 3] = np.asanyarray(p, dtype = float)  
    return T

def apply(T, p):
       return (T @ np.append(p, 1.0))[:3]  #[:3] 利用NumPy切片提取前三个元素x' y' z' 

def R_from_T(T):return T[:3, :3]  # 提取旋转矩阵
def t_from_T(T):return T[:3,  3]  # 提取平移向量

def invert(T):
       R ,t = R_from_T(T), t_from_T(T)
       T_inv_manual = make_transform(R = R.T, p = -R.T @ t) 
       return T_inv_manual

def Rz(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

# 1 纯平移
assert np.allclose(apply(make_transform(p=[1,2,3]), [0,0,0]), [1,2,3]), "纯平移失败"   # assert的意思是 如果错了 输出后面那段话

# 2 纯旋转
assert np.allclose(apply(make_transform(R=Rz(np.pi/2)), [1,0,0]), [0,1,0]), "纯旋转失败"

# 3 旋转 + 平移
assert np.allclose(apply(make_transform(R=Rz(np.pi/2), p=[1,2,3]), [1,0,0]), [1,3,3]), "旋转+平移失败"

# 4 逆矩阵（公式，不许 np.linalg.inv）
T  = make_transform(R=Rz(np.pi/3), p=[2,-1,5])
Ti = invert(T)
assert np.allclose(T @ Ti, np.eye(4)), "逆矩阵错了（是不是写成 -t 了？）"

# 5 左乘右乘
A = make_transform(R=Rz(np.pi/2), p=[1,0,0])
B = make_transform(p=[1,0,0])
assert np.allclose((B @ A)[:3,3], [2,0,0]), "左乘错了"
assert np.allclose((A @ B)[:3,3], [1,1,0]), "右乘错了"

print("✅ 全部通过")