本周的收尾

# 任务一：
手撕 C++ PID
特别注意五个核心机制：
1.微分先行
2.第一拍不冲
3.三项相加
4.输出限幅
5.条件积分抗饱和

# 任务二：
```
写 README

梳理一下arm_control的搭建使用流程：

第一阶段：搭建阶段（包工头视角）
1. 建目录骨架
创建四大核心目录，严格区分接口与实现：
include/arm/：放头文件（.hpp），只写声明。
src/：放源文件（.cpp），写具体实现。
tests/：放单元测试代码（.cpp）。
apps/：放演示程序（.cpp）。

2. 填代码（实现核心模块）

在 include/arm/ 里写出 pid.hpp、lowpass.hpp、kinematics.hpp。定义好 class 的接口和私有变量。绝不能在这里写具体的 if-else 逻辑，也绝不能写 using namespace std;。
在 src/ 里写对应的 pid.cpp、lowpass.cpp、kinematics.cpp。#include 对应的头文件，把具体算法填进去（比如 PID 的限幅和积分抗饱和）。记得函数名前面要加 PID::。

在 tests/ 和 apps/ 里写调用这些头文件的测试和演示代码。

3. 编写构建图纸 CMakeLists.txt
这是整个工程的灵魂，必须写清三件事：

add_library(arm_control STATIC src/pid.cpp src/lowpass.cpp src/kinematics.cpp)：把三个源文件打包成静态库。

target_include_directories(arm_control PUBLIC include)：把 include 目录暴露给别人（必须是 PUBLIC）。

target_link_libraries(test_pid PRIVATE arm_control)：把测试程序和演示程序链接到刚才生成的库上。

4. 编译与测试
执行四步曲：

cmake -S . -B build
make -C build -j
ctest --test-dir build
全部通过后，build/ 目录下就会生成 libarm_control.a 和可执行文件。你的“商品”就出厂了。

第二阶段：使用阶段（客户视角）
假设另一个人拿到了你的库（或者你在另一个新项目里想用你的库），他该怎么调用？

1. 包含头文件
在他的 .cpp 文件里，直接引入头文件：

#include "arm/pid.hpp"
#include "arm/kinematics.hpp"
2. 调用 API
直接实例化对象并调用函数：

arm::PID pid(10.0, 5.0, 5.0, 20.0);
double force = pid.update(10.0, 0.0, 0.01);

Eigen::Matrix4d T = arm::dh(0.3, 0.0, 1.0, 0.0);
3. 配置他自己的 CMakeLists.txt
他不能干瞪眼调用，他的构建系统必须知道你的库在哪。有三种常见接入方式：

方式 A（源码级嵌套）：把你的 arm_control 文件夹直接扔进他的项目里。在他的 CMake 里写：

cmake
add_subdirectory(arm_control)
target_link_libraries(他的目标 PRIVATE arm_control)

方式 B（本地静态库链接）：他拿到你编译出来的 libarm_control.a 和 include 文件夹。在他的 CMake 里写：
cmake
target_include_directories(他的目标 PRIVATE /path/to/include)
target_link_libraries(他的目标 PRIVATE /path/to/libarm_control.a)

方式 C（系统级安装）：你通过 make install 把库和头文件装到了系统的 /usr/local/ 下。他只需要写一行 find_package(arm_control REQUIRED) 就能直接用了。

💡 总结整个流程的核心
你的所有流程，都是围绕这四句话展开的：

写代码：在 include 写声明，在 src 写实现。
编图纸：用 CMake 规定怎么把 src 变成 .a 库。
做质检：在 tests 里跑边界测试，保证算法不炸。
交商品：打包发出去，别人拿着 include 和 .a 就可以顺利调用。
```

# 任务三 自测五道题
Q1 单摆的动力学方程是 τ = m·l²·θ̈ + m·g·l·sinθ
(a) 摆到 θ = 0（竖直向下）保持不动，需要多大力矩？为什么？
需要 0 的力矩。因为当 θ = 0 时，sin(0) = 0，重力项的力矩为 0。此时单摆处于最低点平衡位置，重力不会产生阻碍旋转的力矩
(b) 摆到 θ = 0.5 rad 保持不动，需要多大力矩？（m = l = 1，g = 9.81）
需要 4.703 Nm。代入公式，角加速度 θ''=0，只需要克服重力：所以 τ = 1 * 9.81 * 1 * sin(0.5) ≈ 4.703

Q2 单摆的 A = [[0,1],[-g/l,0]]，特征值是 ±3.1321j。
(a) 从这两个特征值怎么算出「周期 = 2.006067 s」？
特征值为纯虚数 ±jω，其中虚部 ω = 3.1321 就是系统的固有振荡角频率。周期 T = 2π / ω = 2π / 3.1321 ≈ 2.006067 秒
(b) 用前向欧拉离散化（Ad = I + A·dt），dt 取多小才不发散？
不管取多小都会发散。因为对于无阻尼系统（特征值实部为0），前向欧拉离散化后的特征值模长永远是 sqrt(1 + (ωdt)^2)，这个值永远大于 1。所以每一步计算都在凭空放大能量，最终必然导致数值爆炸发散

Q3 你有 16 个 double，按行主序填好了一个 4×4 变换矩阵。现在写：
Eigen::Map<Eigen::Matrix4d> M(buf);

buf[0]=R00,  buf[1]=R01,  buf[2]=R02,     buf[3]=p_x (第一行)
buf[4]=R10,  buf[5]=R11,  buf[6]=R12,     buf[7]=p_y (第二行)
buf[8]=R20,  buf[9]=R21,  buf[10]=R22,    buf[11]=p_z (第三行)
buf[12]=0,   buf[13]=0,   buf[14]=0,      buf[15]=1 (第四行)

(a) M(0,3) 是什么？
Eigen::Matrix4d 的类型定义默认是列主序
即 M(0,3)，它对应的是 buf[12] 是0
(b) 那个本应在 M(0,3) 的平移量跑到哪去了？
按行主序，px应该在 M(0,3)。
但错误矩阵p x 跑到了最后一行第一列，即 M(3,0)
(c) 怎么改才对？
你需要告诉 Eigen 的 Map 模板：“我这块内存不是列主序的，是行主序的，你按行来读！”
C++ 代码必须显式指定完整的模板参数：

Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> M(buf);

加上 Eigen::RowMajor 之后，Eigen 就会按 buf[0], buf[1], buf[2], buf[3] 的顺序填满第一行，从而完美还原出原来的 
T 矩阵。


Q4 同一个 PID，参数 kp=10, ki=5, kd=5, u_lim=20，三种抗饱和模式跑同一个场景。用 gdb 抓到的 max |integral_| 分别是 106.107 / 4.000 / 2.662。
(a) 这三个数分别对应哪种模式？
106.107 对应 None（无抗饱和）；4.000 对应 Clamp（钳位）；2.662 对应 Conditional（条件积分）
(b) 为什么「条件积分」的结果不是 0？
因为条件积分并不是“永远不积分”。在系统还没有达到限幅饱和（即 u == raw）的初期阶段，误差仍然在正常累积积分。2.662 就是它在触碰饱和边界前积累的值。一旦达到饱和，它才停止累加

Q5 你的 PID::update(target, current, dt) 在 dt = 0 时会发生什么？
(a) 默认写法（没有守卫）下，输出是多少？为什么？
输出是 -u_lim_（如 -50）。因为 dt=0 会导致计算微分时除以零，产生 -inf 或 inf（无穷大）。随后这个无穷大在限幅环节被 std::max/min 强行截断成了限幅的下界
(b) 为什么不写 if (dt <= 0.0) 而写 if (!(dt > 0.0))？
为了防御 NaN（非数字）的出现。如果 dt 是 NaN（例如传感器故障），NaN <= 0 的结果永远是 false，无法拦截。但 !(NaN > 0) 的结果是 true，能成功拦截这种异常