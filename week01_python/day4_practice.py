# 1.基本函数
def add(a,b):
    return a + b

def is_even(n): # 判断n是不是偶数
    return n % 2 == 0 # 比较运算符（==）的运算结果是布尔值  故这里返回的是True 或Flase

print(add(4,5))
print(is_even(8))
print(is_even(7))

# 2.默认参数 + 关键字参数
def greeting(name,greeting = "你好"):
    print(f"{name},{greeting}") # 这里没有返回值return  print 是为了“给人看”，return 是为了“给程序用” 

greeting("luoyi")
greeting("zhangsan","hello")
greeting(greeting = "hi",name = "wangwu")

# 3.返回多个值
def min_max(nums):
    return min(nums),max(nums)
# print( low,high = min_max([1,5,3,41,34,9])) 错误
# 赋值操作（low, high = ...）是一个“语句”（Statement），而不是一个“表达式”（Expression）。
# print() 是一个函数，它的括号里只能接受表达式
low,high = min_max([3,7,4,11,9])
print(low,high) # 或者直接print(min_max([3,7,4,11,9]))

# 4.亲手验证不写return会怎么样
def no_return(a,b):
    print(a + b)

r = no_return(1,2) # 这里r实际上根本没有接收到1 + 2的计算结果
print(r) # 输出None

# 5. is None 的用法
def find(nums,target):
    for n in nums:
        if n == target:
            return n

print(find([1,3,6,3],3))
print(find([1,3,6,3],9))

result = find([1,3,5,8,6],9)

if result is None:
    print("没找到")

print(find([1,3,5,8,6],9) is None) #输出 True