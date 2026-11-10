# 从数学公式变成真正的工程代码 昨天的代码是面向过程的写法 下面要学习面向对象的写法
# 连续形式的PID：u = Kp·e + Ki·∫e dt + Kd·de/dt
# 把连续变离散：
# 计算机不懂微积分 要把连续时间切割成无数个时间片段dt

#   连续	离散	代码
#  ∫e dt	累加	integral += e * dt
#  de/dt	差分	derivative = (e - e_prev) / dt
#     dt	循环的时间步长	由调用者给
# 
# 要把上面三行装进一个class里面 PID需要记忆 只有class可以实现
# 为什么PID需要记忆：
# 积分项（I）：它要计算过去的总账（过去所有误差的累加）
# 微分项（D）：它要计算变化率（得拿“现在的误差”减去“上一次的误差”）
# PID 控制器在运行的时候，必须得记住：历史总账 和 上一次误差

# 函数式：状态存在外面，每次都得自己传
# integral, e_prev = 0.0, 0.0
# u, integral, e_prev = pid_step(e, integral, e_prev, dt)
# 你每次调用这个函数，不仅要输入当前的误差（e），还得把之前攒的钱（integral）和上次的账单（e_prev）一起抱过去喂给它

# 类：状态存在对象里
# pid = PID(kp=10, ki=5, kd=5)
# u = pid.update(target, current, dt)      # 干净得多
# 不需要再管 integral 和 e_prev 去哪了！它们在 pid 这个对象内部 这叫做封装