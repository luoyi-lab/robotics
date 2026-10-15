import math

class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def distance_to(self,other):
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx*dx + dy*dy)

    def move(self,dx,dy):
        self.x += dx
        self.y += dy
        # 不需要return 只是执行一个动作（就地修改），就不需要返回任何东西

    def __str__(self):   # 可以直接print(p1) 打印出p1里面的内容  不然只能打印p1的内存地址
        return f"Point({self.x},{self.y})"       # 完全等价于 C++ 里的重载 << 运算符 

    def distance_to_origin(self):
        return self.distance_to(Point(0, 0))

# 1.基本验证
p1 = Point(2,4)
p2 = Point(7,6)
print(p1.distance_to(p2))
print(p2.distance_to(p1))

# 2.验证self的作用
p3 = Point(15,12)
print(p1.x,p1.y)
print(p2.x,p2.y)
print(p3.x,p3.y)

# 3.验证 不用预先声明属性
p3.z = 99
print(p3.z)

# 附加练习
# 1.加一个方法 move(dx, dy)：把点移动 (dx, dy)
# 调用：p1.move(1, 1) → p1 变成 (3, 5)
p1.move(1,1)
print(p1.x,p1.y)

# 2.加一个方法 distance_to_origin(self)：到原点的距离
# 直接复用distance_to
Origin = Point(0,0)
print(p1.distance_to(Origin))

# 3.加一个特殊方法 __str__：
# def __str__(self):
#    return f"Point({self.x}, {self.y})"
#    加完之后 print(p1) 会打出什么？不加的话又打出什么？

print(p1)