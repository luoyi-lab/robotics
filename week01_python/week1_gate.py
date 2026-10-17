# 解法1 基础循环版
def sum_even_squares1(nums):
    # 接受一个数字列表 返回其中所有的偶数的平方
    result = []
    for n in nums:
        if n % 2 == 0:
            result.append(n**2)
    return result

print(sum_even_squares1([1,2,3,4,5]))
print(sum_even_squares1([]))
print(sum_even_squares1([1,3,5]))
print(sum_even_squares1([1,-2,-3,-4,6]))




# 解法2 列表推导法 Pythonic 终极形态
def sum_even_squares2(nums):
    return [n**2 for n in nums if n % 2 == 0]
    # Python 不仅可以直接返回列表，还支持“当场直接生成一个列表并返回”
print(sum_even_squares2([1,2,3,4,5]))
print(sum_even_squares2([]))
print(sum_even_squares2([1,3,5]))
print(sum_even_squares2([1,-2,-3,-4,6]))