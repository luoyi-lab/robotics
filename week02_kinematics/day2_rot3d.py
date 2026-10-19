# 1.3D旋转 绕一根轴转 （多复习机器人学）
# 1.1 绕X轴转  x坐标不变，y-z平面里转：
# Rx(θ) = [[ 1,   0,    0  ],
#         [ 0, cosθ, -sinθ],
#         [ 0, sinθ,  cosθ]]

# 1.2 绕Y轴转  y坐标不变，x-z平面里转：
# Ry(θ) = [[ cosθ, 0, sinθ],
#         [   0,  1,   0 ],
#         [-sinθ, 0, cosθ]]

# 1.3 绕Z轴转  z坐标不变，x-y平面里转：
# Rz(θ) = [[ cosθ, -sinθ, 0],
#         [  sinθ, cosθ,  0],
#         [   0,    0,    1]]

# 2.代码练习
import numpy as np

def Rx(theta):
    # 定义sin cos
    c , s = np.cos(theta) , np.sin(theta)
    return np.array([[1 ,0 ,0 ],
                     [0 ,c ,-s],
                     [0 ,s ,c ]])


    
def Ry(theta):
    c , s = np.cos(theta) , np.sin(theta)
    return np.array([[c ,0 ,s ],
                     [0 ,1 ,0 ],
                     [-s,0 ,c ]])


def Rz(theta):
    c , s = np.cos(theta) , np.sin(theta)
    return np.array([[c ,-s,0 ],
                     [s ,c ,0 ],
                     [0 ,0 ,1 ]])

# 创建三根单位轴向量
x = np.array([1.0,0.0,0.0])
y = np.array([0.0,1.0,0.0])
z = np.array([0.0,0.0,1.0])

# 验证：循环验证1：绕X轴转90° 把Y轴送到Z轴 Z轴送到Y轴反方向
print(Rx(np.pi/2) @ y)
print(Rx(np.pi/2) @ z)

# 循环验证2:绕Y轴转90° 把Z轴送到X轴 X轴送到Z轴反方向
print(Ry(np.pi/2) @ z)
print(Ry(np.pi/2) @ x)

# 循环验证3:绕Z轴转90° 把X轴送到Y轴 Y轴送到X轴反方向
print(Rz(np.pi/2) @ x)
print(Rz(np.pi/2) @ y)

# 验证三个都必须是正交矩阵，且 det = 1  典型错误：print(Rx.T @ Rx)  Rx是一个函数 对Rx取转置是无稽之谈 所以必须要先把矩阵造出来
R_x = Rx(np.pi/2)
R_y = Ry(np.pi/2)
R_z = Rz(np.pi/2)
print(R_x.T @ R_x)
print(R_y.T @ R_y)
print(R_z.T @ R_z)
print(np.linalg.det(R_x))
print(np.linalg.det(R_y))
print(np.linalg.det(R_z))


# 更快的方法一次性把 X、Y、Z 三个旋转矩阵都造出来，然后挨个检验 
for name, R in [("Rx", Rx(np.pi/2)), ("Ry", Ry(np.pi/2)), ("Rz", Rz(np.pi/2))]: # 列表里装元组
    print(name, "正交:", np.allclose(R.T @ R, np.eye(3)), " det:", np.linalg.det(R)) #  np.allclose(R.T @ R, np.eye(3))：与单位阵比较   np.linalg.det(R)：计算行列式

# 转45°试一下
print(Rx(np.pi/4) @ y)
print(Ry(np.pi/4) @ z)
print(Rz(np.pi/4) @ x)