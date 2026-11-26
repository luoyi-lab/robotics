今天之前 arm_control 里面的运动学模块只有单关节变换 今天要将它改为整条臂的两连杆

已有的是这个：W3  day2_fk_homo.py   →  fk_2link(theta1, theta2, L1, L2)   Python 版，返回 2D 位置

# 第一部分 避坑 立规矩
W3的代码如下：
def fk_2link_homo(theta1, theta2, L1=1.0, L2=1.0):
    T = link_transform(theta1, L1) @ link_transform(theta2, L2)
    return T[:3, 3][:2]      # ← 名字叫 homo，返回的却是 2D 坐标

名字写着「齐次变换矩阵」，实际返回的是 (x, y) 两个数。 因为当时只关心位置，但它现在是"约定不一致"的隐患。
所以今天 C++ 版必须先定清楚约定：
项	                     今天定成什么	                                               为什么
角度单位	                 弧度	                                    全库统一，和 dh() / Eigen 一致
返回值	        完整 4×4 齐次变换 Eigen::Matrix4d	         库函数要给全信息，调用方想要位置就 T.block<3,1>(0,3)
参数顺序	       (theta1, theta2, L1, L2)	                          和 Python 版一致，方便 Day 5 对账
坐标系	               dh() 的标准型，T = T1 · T2	                           和 W6 Day 6 一致

# 写代码 改文件
2.1 include/arm/kinematics.hpp —— 加声明

2.2 src/kinematics.cpp —— 实现

2.3 tests/test_kinematics.cpp —— 加断言