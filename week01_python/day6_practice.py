import numpy as np

# 1.创建与查看
a = np.array([[1,2,3],
              [4,5,6]])

print(a.shape) # 输出一个元组 查看这个矩阵多大 几行几列 [2，3]
print(a.dtype) # 查看这个矩阵中的元素类型 int64
print(a.ndim) # 查看这个矩阵的维数 行列 2维
print(a.size) # 查看这个矩阵中元素的数量

# 2.取值
print(a[0]) #输出第0行
print(a[1]) #输出第1行
print(a[1,2]) #输出第1行 第2列
print(a[:,0]) #输出全部行 第0列 
print(a[:,1]) #输出第1列
print(a[1,:]) #输出全部列 第1行 和 print(a[1]) 结果一模一样

# 3.特殊数组
print(np.zeros((2,3))) # 两行三列全为0
print(np.ones((2,3))) # 两行三列全为1
print(np.eye(3)) # 三行三列单位矩阵
print(np.arange(0,10,2)) # 输出一维数组[0,2,4,6,8]（不是元组（元组应该是()）也不是矩阵）

# 4.逐元素 vs 矩阵乘法
A = np.array([[1,2],
              [3,4]])
B = np.array([[5,6],
              [7,8]])
print(A * B) #逐元素相乘
print(A @ B) #矩阵乘法

# 5.转置 
print(A.T)

# 6.布尔索引
b = np.array([10,20,30,40,50,60])
print(b>25) #逐元素比较 返回布尔数组
print(b[b>25]) #把 True 对应位置的数据留下来，把 False 的丢掉

# 7.和list对比
list1 = [1,2,3]
list2 = [4,5,6]
print(list1 + list2) # list是拼接
print(A + B) #逐元素相加