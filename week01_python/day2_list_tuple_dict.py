#1.python中列表list = C++的vector

# C++ 动态数组要这样写
# vector<int> nums = {3, 1, 4, 1, 5};
# nums.push_back(9);
# cout << nums[0];        // 3
# cout << nums.size();    // 6

#Python list 列表 如下：
nums = [3,1,4,1,5]
nums.append(9)#在尾部加
print(nums[0])
print(len(nums)) #输出长度
#注意：python中list可以混装不同类型 [1, "abc", True]
# len(nums)是python的内置函数

#下标 注意负号 这是python独有的
nums = [10,20,30,40,50]
print(nums[0]) #输出第一个
print(nums[-1]) #输出倒数第一个
print(nums[-2]) #输出倒数第二个

#切片 python最爽的东西
nums = [10,20,30,40,50]

print(nums[1:3]) #从1到3 不含3
print(nums[:3])  #从0开头到3 含3
print(nums[2:]) #从2到尾 含尾

print(nums[::2]) #10, 30, 50 从第一个开始 每隔一个
print(nums[::-1]) #倒序
#[起点:终点:步长]

#增删改
nums = [10,20,30]
nums.append(40)  # 尾部加40
nums.insert(0,5) # 在下标0处插入5
nums.remove(20) # 删掉「值为20」的第一个
last = nums.pop() # 弹出最后一个(列表中最后一个位置被删掉)，并返回它 即可以print(last)
#remove 删的是「值」，pop 删的是「位置」
nums.remove(10) #找"20"这个值，删掉它 
nums.pop(0) #删掉位置0上的东西

#2.元组 tuple = 建好就不能修改的list tuple如下：
point = (3,5)
print(point[0])
# point[0] = 9 错误 因为tuple不可以更改

#3.字典 dict = C++ 的map
# C++
# map<string, int> ages;
# ages["luoyi"] = 22;
# ages["zhang"] = 30;
# python的dict如下：{键: 值}
ages = {"luoyi":22 , "zhangxiang":32}
ages["wangji"] = 23 #[]新增
print(ages["luoyi"])  #输出22

# 常用操作
ages = {"luoyi":22 , "zhangqi":26}

print(ages.keys()) #输出所有键
print(ages.values()) #输出所有的值
print("luoyi" in ages) #直接用in关键字查键在不在 输出True
print(ages.get("nobody",0)) # .get() 方法安全取值。如果 nobody 不存在，返回默认值 0，不会报错，也不会修改原始字典。
# 和C++ map最大的区别
# C++：访问不存在的键 → 自动创建一个默认值！
# map<string,int> m;
# cout << m["nobody"];     // 打出 0，而且 m 里真的多了一个 "nobody"
# Python：访问不存在的键 → 直接报错
# ages = {"luoyi": 25}
# print(ages["nobody"])    # ❌ KeyError
# print(ages.get("nobody")) # ✅ None，不报错


# 遍历 同时拿到键和值
for name , age in ages.items():
    print(f"{name}今年{age}岁") #在 Python 里，代码块（循环体、条件体等）不需要大括号 {}，而是靠“缩进”来表示
# for ... in ... 这是 Python 的遍历循环语法
# ages.items() .items() 是字典的一个方法。它把字典里的数据，转换成了一种叫“键值对视图”的东西 里面打包了所有键和值
# name , age name, age 会自动把这个元组拆开，把第一个元素 "luoyi" 交给 name，把第二个元素 25 交给 age