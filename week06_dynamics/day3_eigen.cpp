/*从“理论推导”跨入“真实机器人C++工程”
在 Python 里，有 numpy 帮你打理一切。但在 C++ 里，如果你自己手写 for 循环去做矩阵乘法，代码不仅丑，而且慢到令人发指
所以，全世界的机器人和自动驾驶公司都在用同一个救星：Eigen 库

第一部分 Eigen是什么
它是什么：它是一个 C++ 的纯头文件库（Header-only）。

区别：平时你装库（比如 OpenCV）需要编译出 .so 或 .dll 文件。
Eigen 没有这些，它的源码全写在 .h 文件里。所以在 C++ 里用 #include 引入它之后，
编译时加上 -I/usr/include/eigen3 就能直接运行。极其轻量！

当你在自己的 .cpp 文件里写下 #include <Eigen/Dense> 时，C++的预处理器会直接把这个头文件里的所有代码（哪怕几万行）原封不动地“复制粘贴”到你的文件里。
也就是说，当你的代码被编译时，Eigen 的代码是跟你自己的代码一起被编译的，因此根本不需要单独的 .so 或 .dll 文件来链接

为什么要用它：看图里的对比。你在 W3-W5 写 C++，如果想算一个 4x4 矩阵乘法，得手写三层 for 循环；
用 Eigen，只需一行 M1 * M2。更牛逼的是，它支持向量化（SIMD），能利用 CPU 硬件直接把 4 个浮点数一起算，速度起飞。

两种矩阵的行数和列数尺寸：

类型	                                   尺寸什么时候知道	        内存         	什么时候用
Matrix4d / Matrix3d / Vector3d	               编译期	           栈	  变换矩阵、位置、轴 —— 永远 4×4 / 3×3
MatrixXd / VectorXd	                           运行期	           堆	    关节数 n 是变量时（雅可比 6×n）

Matrix4d（固定尺寸）—— 提前定好的快餐盒
编译期：你写出 Matrix4d 的时候，编译器在把代码翻译成机器码的时候，就已经知道这是一个 4 行 4 列的矩阵。
好处：因为提前知道了大小，电脑会在内存的“栈（Stack）”里给它留一个固定的位置。
栈的速度极快，就像你吃快餐时，直接把饭菜打进标准化的餐盘里，拿起来就吃。
机器人里的死理：在 3D 空间里，你要描述一个物体的“位置（x,y,z）”和“姿态（旋转）”，把它们拼在一起，永远、绝对、刚好就是一个 4 行 4 列的齐次变换矩阵
不管你是在算大疆无人机、波士顿机器狗还是工厂里的机械臂，这个规矩永远不变。所以，只要遇到位置姿态转换，一律无脑用 Matrix4d，因为尺寸是死定的，最快

MatrixXd（动态尺寸）—— 临时申请的伸缩饭盒
运行期：你写 MatrixXd 的时候，编译器根本不知道它有多大。直到程序真正跑起来（运行期），程序接收到用户的输入，它才会去内存的“堆（Heap）”里临时申请一块空间
坏处：每次用的时候都要临时去分配空间，用完还要释放，这个过程叫“分配开销”，非常浪费时间
机器人里的应用：什么时候用？比如你要存储一个机械臂 6 个关节的受力数据。A 公司的机械臂可能是 6 关节，B 公司可能是 7 关节
你写代码的时候根本不知道用户会用哪个，所以只能等程序运行起来，读到了具体的关节数n，再动态生成一个n×n 的矩阵

第二部分 四件套和两个致命坑
核心四件套（必须掌握）：M，就是“Matrix”（矩阵）
1.M.transpose()   转置
2.M.inverse()   求逆   极慢且数值不稳，平时慎用
3.M.lu().solve(b)  解方程神器。用来解 Ax=b
4.M.isApprox(N)  浮点数判等。在计算机里，0.1 + 0.2 != 0.3。永远不要用 == 去比较两个矩阵，必须用 isApprox，判断它们的差值是否在极小范围（比如 1e-9）内

坑1：auto 关键字导致的数据污染
在 Eigen 里，auto t = M.transpose(); 绝对不能用！

假设你的矩阵 M 是一块黑板，上面写满了数据
M.transpose() 做了什么？
在数学上，转置是把行列互换。在 Eigen 眼里，它并没有立刻搬一把椅子去把黑板上的数据重新抄一遍。相反，它只在黑板旁边贴了一张小纸条，上面写着：“请看这块黑板的转置。”
在编程里，这叫“表达式（Expression）”或者“视图（View）”

auto t = M.transpose(); 做了什么？
你用了 auto，C++ 编译器会去推导 M.transpose() 的类型。它推导出的类型不是矩阵，而是 Eigen::Transpose（转置视图）
也就是说，t 并不是一个新的矩阵，它只是死死抱住了黑板旁边的那个“转置标签”

灾难降临（数据污染）：
你接着执行了 M(0,0) = 99;（修改了黑板上的第一个数字）
按道理原矩阵变 和t没有关系 但是：
此时，你再用 t(0,0) 去读数据，读出来的是 99
为什么？因为 t 指向的还是那块黑板！黑板变了，贴在旁边的标签也变了。你以为你保存了一份转置副本，实际上你只是保存了一个随时会变的“引用”

终极解法：只要在 Eigen 里做矩阵运算，永远不要用 auto 去接收结果！
要强制指定类型，让Eigen在新内存上老老实实的写：
Eigen::Matrix4d t = M.transpose();     // 指定一个4×4矩阵 是M的转置

坑 2：列主序（Column-Major）
在 C++ 的多维数组里，通常是按行存储的（先存第一行，再存第二行）
但在数学和 Eigen 中，默认是按列存储的（先存第一列，再存第二列） 例子如下：
内存里躺的 16 个 double：

  [0] [1] [2] [3] [4] [5] [6] [7] [8] [9] [10] [11] [12] [13] [14] [15]

Eigen 的读法（列主序）：
  ┌[0] [4] [8]  [12]┐
  │[1] [5] [9]  [13]│
  │[2] [6] [10] [14]│
  └[3] [7] [11] [15]┘

这会导致什么：如果你把一个按行顺序填满的数组，强行喂给默认的 Eigen 矩阵，它读出来的就是转置后的矩阵！

任务 B & C：Map 零拷贝魔法
真实场景：如果上游给你发来一个 C 语言风格的一维数组 double buf[16]，
里面存着机械臂传来的矩阵数据。如果用 Matrix4d M = buf;，Eigen 会开辟一块新内存，把数组复制进去，很慢

零拷贝 Map：Eigen::Map<Eigen::Matrix4d> M(buf)。它不复制，而是直接在原数组的内存上套一个壳。你对 M 的修改，实际上就是在修改原数组 buf

实操陷阱：如果 buf 是按照行填进去的，你用默认的 Map<Matrix4d> 读出来，就会变成“转置”的矩阵（因为默认是列主序）。这时候你需要明确告诉 Eigen：我这是个行主序的矩阵，即 Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> M_row(buf);。

任务 C：改 buf，M 跟着变；改 M，buf 跟着变。这就证明了它是零拷贝。而如果你用 Matrix4d Copy = M_row;，再改 Copy，buf 就不会变，这就证明了这是真实的深拷贝

代码训练：
*/
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;

// 任务A 齐次变换与闭式逆
// 使用标准型D-H参数构造4×4变换矩阵
Matrix4d dh(double theta, double d, double a, double alpha)
{
    Matrix4d T = Matrix4d::Identity();  // 创建一个名为 T 的 4x4 矩阵，并将其初始化为单位矩阵

    // 按照教程给的顺序：T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    // 虽然可以通过手写 4 个矩阵连乘来实现，但在 Eigen 里用 AngleAxisd 和 Translation3d 更优雅
    T = (AngleAxisd(theta, Vector3d::UnitZ())  // 表示：绕 Z 轴（Vector3d::UnitZ() 即向量 (0,0,1)）旋转 theta 角度
        * Translation3d(0, 0, d)  // 表示：沿 X 平移 0，沿 Y 平移 0，沿 Z 平移d
        * Translation3d(a, 0, 0)  // 沿 X 轴平移a
        * AngleAxisd(alpha, Vector3d::UnitX())).matrix(); // 表示绕X轴旋转alpha角度
        return T;
}

// 闭式逆 针对 4x4 齐次变换矩阵的特殊求逆法
Matrix4d fast_inverse(const Matrix4d& T)
{
    Matrix4d T_inv = Matrix4d::Identity();
    
    // 1. 提取旋转矩阵 R 和位置向量 p
    Matrix3d R = T.block<3, 3>(0, 0);
    Vector3d p = T.block<3, 1>(0, 3);
    
    // 2. 闭式逆公式：R^T 就是转置，位置变成 -R^T * p
    T_inv.block<3, 3>(0, 0) = R.transpose();
    T_inv.block<3, 1>(0, 3) = -R.transpose() * p;
    
    return T_inv;
}

int main() 
{
    cout << "===== 开始 Day 3 Eigen 测试 =====" << endl;

    // ----------------------------------------
    // 任务 A 测试用例
    // ----------------------------------------
    cout << "\n--- 任务 A: 齐次变换 + 闭式逆 ---" << endl;
    
    // 杆 1 的参数：θ=0.3, d=0, a=1.0, α=0
    Matrix4d T1 = dh(0.3, 0.0, 1.0, 0.0);
    // 杆 2 的参数：θ=0.7, d=0, a=1.0, α=0
    Matrix4d T2 = dh(0.7, 0.0, 1.0, 0.0);
    
    // 末端总变换
    Matrix4d T = T1 * T2;
    cout << "T = T1 * T2 矩阵:\n" << T << endl;
    
    // 提取末端位置 (前三行，第四列)
    Vector3d pos = T.block<3, 1>(0, 3);
    cout << "末端位置 (pos): " << pos.transpose() << endl;
    
    // 提取末端角度 (Z轴上的角度累积)
    double end_theta = atan2(T(1, 0), T(0, 0)); 
    cout << "末端角度: " << end_theta << " rad" << endl; // 应该是 1.0 rad
    
    // 测试闭式逆
    Matrix4d T_inv_closed = fast_inverse(T);
    // 用 Eigen 自带的通用逆来做对比
    Matrix4d T_inv_standard = T.inverse();
    
    // 计算两者的误差（差距极小说明正确）
    double inv_diff = (T_inv_closed - T_inv_standard).norm();
    cout << "闭式逆与通用逆的误差: " << inv_diff << " (参考: < 1e-9)" << endl;
    
    // 验证 T * T^-1 = I
    double identity_diff = (T * T_inv_closed - Matrix4d::Identity()).norm();
    cout << "T * T^-1 与单位矩阵的误差: " << identity_diff << " (参考: < 1e-12)" << endl;

    // 任务B 零拷贝
    cout << "\n--- 任务 B & C: Map 零拷贝与拷贝对比 ---" << endl;

    // 照着 T 的数值，按【行主序】填入 C 风格数组
    double buf[16] = 
    {
        0.540302, -0.841471, 0.0, 1.495639,
        0.841471,  0.540302, 0.0, 1.136991,
        0.0,       0.0,      1.0, 0.0,
        0.0,       0.0,      0.0, 1.0
    };

    // 1. 默认列主序读取（把 buf 当成列主序）
    Map<Matrix4d> M_col(buf);
    cout << "M_col 默认列主序读取:\n" << M_col << endl; 
    // 你会发现 M_col(0,3) 变成了 0.0，而 1.495639 跑到了 M_col(3,0)
    
    // 2. 指定行主序读取（告诉 Eigen 这个内存是按行排的）
    Map<Matrix<double, 4, 4, RowMajor>> M_row(buf);
    cout << "M_row 指定行主序读取:\n" << M_row << endl;
    // 这个读出来的才是正确的 T 矩阵

    // 任务C 证明零拷贝 / 证明拷贝
    cout << "\n--- 验证零拷贝 ---" << endl;

    // A. 改原数组，矩阵跟着变（零拷贝成立）
    buf[0] = 99.0;
    cout << "修改 buf[0] = 99.0 后，M_row(0,0) = " << M_row(0,0) << " (零拷贝成立)" << endl;
    
    // B. 改矩阵，原数组跟着变（零拷贝成立）
    M_row(1, 1) = -7.0;
    cout << "修改 M_row(1,1) = -7.0 后，buf[5] = " << buf[5] << " (零拷贝成立)" << endl;
    
    // C. 真实的拷贝（改拷贝体，原数组不变）
    Matrix4d Copy = M_row;
    Copy(0,0) = 123.0;
    cout << "修改 Copy(0,0) = 123.0 后，buf[0] = " << buf[0] << " (说明 Copy 是独立的内存)" << endl;

    return 0;
}