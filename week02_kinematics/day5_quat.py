# 四元数 Quaternion
# 1.为什么需要四元数
# 欧拉角的三个毛病：
# （1）万向锁：pitch = 90° 时丢一个自由度 
# （2）解不唯一
# （3）插值会跳 比如从-179°到179° 明明只有两度 但会绕一大圈
# 四元数无万向锁 插值平滑 运算快 代价是没有欧拉角直观
# 
# 2.四元数表示方法
# 形式：q = (w, x, y, z)
# 表示旋转的四元数必须是单位四元数
# w² + x² + y² + z² = 1
# 物理含义（要记）：
# 绕单位轴 axis = (ax, ay, az) 转θ角
# q = (cos(θ/2), ax·sin(θ/2),  ay·sin(θ/2),  az·sin(θ/2))

# 例子：绕z轴转90°
# q = (cos(45°), 0, 0, sin(45°)) = (0.7071, 0, 0, 0.7071)

# 3.怎么用四元数转一个点
# 就是要把四元数转换为一个旋转矩阵 公式如下，直接用：
# def quat_to_matrix(q):
#   w, x, y, z = q      把四元数拆成四个分量
#    return np.array([
#        [1-2*(y*y+z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
#        [2*(x*y + w*z),   1-2*(x*x+z*z),   2*(y*z - w*x)],
#        [2*(x*z - w*y),   2*(y*z + w*x),   1-2*(x*x+y*y)],
#    ])
#
# 顺序陷阱：
# 教学/机器人教材： 分量顺序为(w, x, y, z)
# scipy/Ros/大多数库： 分量顺序为(x, y, z, w)
# 同一组 4 个数，顺序反了就是完全不同的旋转，致命的是计算机不会报错，很难发现

# 4.代码训练
import numpy as np

def quat_to_matrix(q):     #把四元数转换为旋转矩阵的函数
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
                     [2*(x*y + w*z),   1-2*(x*x+z*z),   2*(y*z - w*x)],
                     [2*(x*z - w*y),   2*(y*z + w*x),   1-2*(x*x+y*y)],])

def quat_from_axis_angle(axis, theta):     # 接受轴和旋转角度两个参数 绕axis转theta（弧度）如何转换为四元数->(w, x, y, z)
    axis = np.asarray(axis, dtype = float) # 把传入的axis（可能是普通列表）转换为NumPy数组 并且强制指定为浮点数
    axis = axis/np.linalg.norm(axis)   # 归一化，确保是单位轴  np.linalg.norm(axis)是计算轴向量的长度 axis除以这个长度 就变成了单位轴
    half = theta/2  # 取半角
    return np.concatenate([[np.cos(half)], axis * np.sin(half)])
    # np.cos(half) 算出来就是 w （实部）外面加[]是为了让它变成数组格式
    # axis * np.sin(half)  注意到这是 * ，算出来就是x, y, z （虚部）
    # np.concatenate  把前面1个和后面3个数字硬拼成一个长度为4的数组 即得到[w, x, y, z]

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s,0],
                     [s, c,0],
                     [0, 0,1]])

# (1) 单位四元数 得到 单位矩阵
q_id = np.array([1.0, 0.0, 0.0, 0.0])
print(np.round(quat_to_matrix(q_id), 3))

# (2) 绕 z 轴转90°：四元数vs旋转矩阵 必须一致
q = quat_from_axis_angle([0, 0, 1], np.pi/2) # 输入绕的轴以及旋转的角度
print(np.round(q, 4)) # 输出四元数：[0.7071, 0, 0, 0.7071]
print(np.round(quat_to_matrix(q), 3)) # 把四元数转为矩阵
print(np.round(Rz(np.pi/2), 3)) # 直接算的矩阵和上一行通过四元数转换为矩阵的两个结果相同
print(np.allclose(quat_to_matrix(q) , Rz(np.pi/2))) # True

# (3) 绕任意轴转 以  绕(1, 1, 0) 转60° 为例
q2 = quat_from_axis_angle([1, 1, 0], np.pi/3)
R2 = quat_to_matrix(q2)
print("\n绕(1, 1, 0)转 60° 的旋转矩阵为：\n",np.round(R2, 3) )

# (4) 以上得到的旋转矩阵是否合法
print(np.allclose(R2 @ R2.T , np.eye(3))) # True 判断是否正交
print(np.linalg.det(R2)) # 1.0 判断模是否为1

# (5) 半角的影响：转360°会得到什么
q360 = quat_from_axis_angle([0, 0, 1], 2*np.pi)
print("\n转 360° 的四元数:", np.round(q360, 4))       #  [-1, 0, 0, 0]
print("转成矩阵:\n", np.round(quat_to_matrix(q360), 3))  # 还是单位矩阵
# 四元数 [-1, 0, 0, 0] 和 [1, 0, 0, 0] 代进去，算出来的 3x3 矩阵一模一样，全是完美的单位矩阵
# 最核心的物理本质：q 和 -q 是同一个姿态
# 所以绝对不能直接用 == 去判断两个四元数是否相等

# 整个利用四元数的思路 拿到一个轴和一个旋转角度 先写出对应的四元数 然后再把这个四元数转换为旋转矩阵