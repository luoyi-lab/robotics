#重写
name = "luoyi"
#age  = 22
school = "东南大学"

print (f"我的名字是{name}")

#除法
a = 7
b = 2
print (a + b)
print (a / b) #真除数
print (a // b) #整除
print (a % b) #取余
print (a ** b) #幂运算
print (-7 // 2) #向下取整
print (-7 % 2) #结果跟着除数走

#字符串碰数字
age = 22
#print ("我今年"+22) #错误 +两边都是一个类型才能加
print ("我今年"+str(age))
print (f"我今年{age}")
print (f"{7+2}")
print(f"{age > 18}")
#f-string 什么时候有用？当你要把值塞进一句话里的时候
#{} 里放任何"能算出一个值"的东西： 变量能、运算能、比较能、函数调用也能

print(True + True) #Python 的 bool 其实是 int 的子类 两个True相加为2