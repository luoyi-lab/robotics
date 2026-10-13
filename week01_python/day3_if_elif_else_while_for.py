# 1. if/elif/else
# C++ 中：
# if (score >= 90) 
# { 
#     cout << "优秀"; 
#  }
# else if (score >= 80) 
# { 
#     cout << "良好"; 
# }
# else 
# { 
#     cout << "及格"; 
# }
# python 中：
score = 100
if score >= 90:
    print("优秀")
elif score >=80:
    print("良好")
else:
    print("及格")

# 在 C++ 里，if 条件通常只能是 true、false、
# 或者能隐式转成布尔值的数字（比如 0 是假，非 0 是真）和指针（nullptr 是假）

# 但在 Python 里，任何东西都可以直接在 if 里判断“真假”。Python 规定：
# 当“假”用（Falsy）—— 也就是相当于 False：
# False
# None（相当于 C++ 的 nullptr）
# 数字的零：0、0.0
# 空的字符串：""
# 空的列表：[]
# 空的字典：{}

# 当“真”用（Truthy）—— 也就是相当于 True：
# 非零的数字（包括负数）
# 非空的字符串（哪怕只有一个空格 " "）
# 非空的列表、字典、元组

nums = 0
if nums:         # 判断为真  C++ 里 if (v) 对 vector 是编译错误，Python 里 if nums: 就是"列表非空"
    print("有东西 非空")
else:
    print("没东西 空的")

#2. while
i = 0
while i < 5:
    print(i)
    i += 1

#3. for
# 3.1 直接遍历
nums = [3,1,5,6,7,2,9,10,13]
for n in nums:
    print(n)

# 3.2 需要下标 用enumerate
for i,n in enumerate(nums): #enumerate() 函数会把列表变成 (索引, 元素) 的元组对
    print(f"第{i}个是{n}")   #for i, n in enumerate(nums) 直接把它拆开，i 是索引，n 是元素

# 3.3 只要一串数字 用range
for i in range(5): # 0 1 2 3 4
    print(i) 

for i in range(1,6): # 1 2 3 4 5
    print(i)

for i in range(1,6,2):# 1  3  5
    print(i) 

# 3.4 break/contiue
for i in range(10): # 输出的是下标
    if i == 3:
        continue # 跳过下标3
    if i == 7:
        break
    print(i)

for i,n in enumerate(nums): # 输出的是元素
    if i == 3:
        continue
    if i == 7:
        break
    print(n)

# 4 print 的 end
print("a")
print("b")
print("a",end="")
print("b",end="")
# print() 默认每次打完自动换行。加 end="" 就能不让它换行
print()
# 常见用法
# 制表符
print("姓名",end="\t")
print("年龄",end="\t")
print("学校")
print("*" * 30)
print("Lvan",end="\t")
print("18",end="\t")
print("东南大学")

print("a",end=" ") # 加个空格
print("b",end="")
print() # 换行