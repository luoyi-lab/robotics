# 九九乘法表 典型的嵌套循环

# 外层循环 共九行
for i in range(1,10): # 含头不含尾1-9
    # 内层循环 第一行只有一列 第二行有两列 以此类推 
    for j in range(1,i+1): #含头不含尾
        print(f"{j} × {i} = {j*i} ",end="\t")
    print() # 一列打印完了 换行

# 附加练习
# 1.猜数字：while 循环 + break + input()
# 思路：设一个目标数，无限循环让用户输入，猜对了用 break 跳出循环。
target = 19
while True:
    guess = int(input("请输入您猜的数字")) # input(提示语) 的作用：1.在屏幕上打印提示语。 2.等待输入 3.核心：无论输入什么，它永远只返回字符串（str）类型！
    if guess == target:
        print("恭喜你 猜对了")
        break # 猜对，跳出整个循环
    elif guess > target:
        print("猜大了 再试试")
    else:
        print("猜小了 再试试")

#2. 打印 1~100 里的偶数 体会两种写法
#写法一 用 range 的步长 更 Pythonic 
for i in range(2,101,2):
    print(i)

#写法二 用 if 判断
for i in range(1,101):
    if i % 2 == 0:
        print(i)

#3.手动循环实现 sum()
nums = [3, 1, 4, 1, 5]
total = 0
for n in nums: # i是下标 n是元素 规定的
    total += n
print(total)    


