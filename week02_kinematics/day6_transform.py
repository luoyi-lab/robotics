# 齐次变换（非常重要）
# 1.为什么需要齐次变换
#  描述一个位姿变幻 是旋转加上平移：
#  p' = R @ p + t        ← 平移是"加法"，不是"矩阵乘法"
# 痛点来了：如果你有 10 个变换连续做，公式就会变成 R3(R2(R1p+t1)+t2)+t3，全是括号和加号，算起来极其恶心，也极其容易写错
# 数学家为了让旋转和平移统一 将原本的3D的点p强行升维，末尾补了个1 变成了(x, y, z, 1)
# ⎡p'⎤   ⎡ R  t ⎤ ⎡p⎤
# ⎣1 ⎦ = ⎣ 0  1 ⎦ ⎣1⎦
# 第一行的运算： p' = R × p + t    第二行纯粹为了凑格式
# 这样加号就消失了 变成了纯纯的一次矩阵乘法

# 2.4×4的结构        T：齐次变换矩阵（位姿矩阵）
# T = ⎡ R   t ⎤        R 是 3×3 旋转矩阵
#     ⎣ 0   1 ⎦        t 是 3×1 平移向量
#                       最后一行固定是 [0 0 0 1]
# ⎡p'⎤   ⎡ R  t ⎤ ⎡p⎤
# ⎣1 ⎦ = ⎣ 0  1 ⎦ ⎣1⎦

# 3.复合运算：左乘与右乘的区别  和前面学的一模一样
# 左乘：T_new @ T_old    旋转和平移都在世界坐标系下
# 右乘：T_old @ T_new    旋转和平移都在自身坐标系下
# 直观实验见代码训练      实际用处:串联机械臂的正运动学是一路右乘

# 4.逆变换 记住公式就行 逆变换是"先转回去，再反向平移"，而反向平移是在旋转后的坐标系里量的
# T⁻¹ = ⎡ Rᵀ   -Rᵀ @ t ⎤
#       ⎣ 0       1    ⎦      可以由  T @ T⁻¹ = I  推导而来

# 5.代码训练
import numpy as np

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s,0],
                     [s, c,0],
                     [0, 0,1]])

# 构造 4×4 齐次变换矩阵
def T_matrix(R = None , p = None):    # 这里的R和p分别为旋转向量和平移向量  R=None, p=None 是默认参数。意思是，调用这个函数时，你可以只传旋转，也可以只传平移，或者两个都不传
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R    # T[:3, :3]  表示前3行（索引为0，1，2） 前3列
    if p is not None:
        T[:3, 3] = np.asanyarray(p, dtype = float)  # T[:3, 3]  表示前3行 第4列（索引为3）
        # np.asanyarray(p, dtype = float)  把传入的 p（可能是普通列表 [1,0,0]）强制转换成浮点型 NumPy 数组   这一操作R不需要 因为NumPy会自动完成隐式转换
    return T

# 3D点经过T变换
def transform_point(T, p):
    return (T @ np.append(p, 1.0))[:3]  #[:3] 利用NumPy切片提取前三个元素x' y' z' 直接抛弃最后一个1 即返回一个正常的3D坐标

def R_from_T(T):return T[:3, :3]  # 提取旋转矩阵
def t_from_T(T):return T[:3,  3]  # 提取平移向量

p = np.array([1.0, 0.0, 0.0])

# （1）纯旋转
T_rot = T_matrix(R = Rz(np.pi/2))
print(np.round(transform_point(T_rot, p), 3))

# （2）纯平移
T_trans = T_matrix(p = [1, 2, 3])
print(np.round(transform_point(T_trans, p), 3))

# （3）旋转+平移
T = T_matrix(R = Rz(np.pi/2), p = [1 ,2 ,3])
print(np.round(transform_point(T, p), 3))

# （4）复合 = 逐步变换
T1 = T_matrix(R = Rz(np.pi/2), p = [1, 0 ,0])
T2 = T_matrix(p = [0 ,2 ,0])
step = transform_point(T2, transform_point(T1, p))  # 分步变换
once = transform_point(T2 @ T1, p) # 一次性变换
print(np.allclose(step, once)) # True 两者完全一致

# （5）逆变换
T_inv_np = np.linalg.inv(T)  # 计算机自己算的逆矩阵
R, t = R_from_T(T), t_from_T(T)
T_inv_manual = T_matrix(R = R.T, p = -R.T @ t)  # 手算的逆矩阵

print(np.allclose(T @ T_inv_np, np.eye(4))) # True
print(np.allclose(T @ T_inv_manual, np.eye(4))) #True 验证计算机算和手算的矩阵是否都是正确的逆矩阵

# （6）用-t会出错
T_inv_wrong = T_matrix(R= R.T, p = -t)
print(np.allclose(T @ T_inv_wrong, np.eye(4))) #False 手算了错误的逆矩阵

# （7）左乘 vs 右乘
A = T_matrix(R = Rz(np.pi/2), p = [1, 0, 0])
B = T_matrix(p = [1, 0, 0])
left, right = B @ A, A @ B
print(t_from_T(left))
print(t_from_T(right))
print(np.allclose(left, right)) # False 左乘右乘的结果不一样

print(np.round(R_from_T(left), 3))
print(np.round(R_from_T(right), 3))
print(np.allclose(R_from_T(left), R_from_T(right))) # 因为B是纯平移 所以左乘右乘得到的旋转矩阵是一样的