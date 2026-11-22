# arm_control 基础控制库

面向机械臂的底层控制基础库，包含三个模块：

- **PID** —— 带抗饱和（条件积分 / 钳位）的 PID 控制器
- **LowPassFilter** —— 一阶低通滤波器
- **kinematics** —— 基于 Eigen 的标准型 D-H 齐次变换 + 齐次矩阵闭式逆

---

## 目录结构

```text
arm_control/
├── CMakeLists.txt
├── include/arm/     # 公共头文件：只放声明            （菜单）
├── src/             # 实现：编进 libarm_control.a     （后厨）
├── tests/           # 单元测试：边界、断言            （质检）
└── apps/            # 演示程序                        （买家秀）
```

**为什么声明和实现要分开**：别人只 `#include` 你 `include/` 里的头文件，不用碰 `src/`。
改了 `src/` 里一行实现，使用你库的项目**不需要重新编译它们的源码**，只要重新链接。

---

## 怎么编译

```bash
cd arm_control
cmake -S . -B build
make -C build -j
```

四步里的每个参数：

| 参数 | 意思 |
|---|---|
| `-S .` | Source：源码目录是当前目录 |
| `-B build` | Build：构建产物放进 `build/`（和源码分开，方便一键清理） |
| `-C build` | 让 `make` 先 `cd` 到 `build/` 再找 Makefile |
| `-j` | 并行编译，用满所有 CPU 核心 |

编完之后 `build/` 里会有：

```text
build/libarm_control.a      # 静态库本体
build/apps/demo             # PID 演示
build/apps/eigen_demo       # 运动学演示
build/tests/test_pid
build/tests/test_lowpass
build/tests/test_kinematics
```

---

## 怎么跑演示

```bash
./build/apps/demo
./build/apps/eigen_demo
```

`demo` 打印三组抗饱和模式下的超调对比；`eigen_demo` 算一次两连杆 4×4 变换。

---

## 怎么跑测试

```bash
ctest --test-dir build --output-on-failure
```

也可以单独跑某一个：

```bash
./build/tests/test_pid
./build/tests/test_lowpass
./build/tests/test_kinematics
```

| 测试 | 覆盖什么 |
|---|---|
| `test_pid` | 纯 P、第一拍微分不冲、微分先行、双向限幅、条件积分 vs 无抗饱和、reset、`dt≤0` 守卫 |
| `test_lowpass` | `alpha = dt/(tau+dt)`、第一拍直通、递归式闭式解、稳态、3τ、reset、`tau≤0` 守卫 |
| `test_kinematics` | 两连杆末端位姿、负角度、α≠0 的三维情形、闭式逆 vs 通用逆、二次求逆 |

---

## 怎么用（最小示例）

```cpp
#include "arm/pid.hpp"
#include "arm/kinematics.hpp"
#include <cstdio>

int main() {
    // PID
    arm::PID pid(10.0, 5.0, 5.0, 20.0);          // kp, ki, kd, u_lim
    double u = pid.update(1.0, 0.0, 0.01);        // target, current, dt
    std::printf("u = %f\n", u);

    // 运动学：两连杆，标准型 D-H
    const Eigen::Matrix4d T = arm::dh(0.3, 0.0, 1.0, 0.0)
                            * arm::dh(0.7, 0.0, 1.0, 0.0);
    std::printf("末端 = (%f, %f, %f)\n", T(0,3), T(1,3), T(2,3));

    // 闭式逆
    const Eigen::Matrix4d Ti = arm::fast_inverse(T);
}
```

输出：

```text
u = 10.000000
末端 = (1.495639, 1.136991, 0.000000)
```

### 不用 CMake、直接用 g++ 编

```bash
g++ -std=c++17 -Iinclude -I/usr/include/eigen3 \
    your_main.cpp build/libarm_control.a -o your_main
```

> ⚠️ **`-I/usr/include/eigen3` 不能省。**
> `include/arm/kinematics.hpp` 里 `#include <Eigen/Dense>`，而 Eigen 的头文件不在默认搜索路径里。
> 走 CMake 时 `find_package(Eigen3)` 会自动把这个路径加上，裸 `g++` 则必须手写。

---

## 在别人的项目里接入

**方式 A：源码级嵌套**（最简单，把整个文件夹放进他的项目）

```cmake
add_subdirectory(arm_control)
target_link_libraries(他的目标 PRIVATE arm_control)
```

**方式 B：链接编译好的静态库**（只有 `.a` 和 `include/`）

```cmake
target_include_directories(他的目标 PRIVATE /path/to/arm_control/include)
target_link_libraries(他的目标 PRIVATE /path/to/arm_control/build/libarm_control.a)
```

> 方式 B 还要自己保证 Eigen 可用：`find_package(Eigen3 REQUIRED)` + `target_link_libraries(他的目标 PRIVATE Eigen3::Eigen)`。
> 因为 `arm_control` 是 `PUBLIC` 依赖 Eigen 的 —— 这个「传播」只在方式 A 里自动生效。

---

## API

### `arm/pid.hpp`

```cpp
namespace arm {
enum class AntiWindup { None, Clamp, Conditional };

class PID {
public:
    PID(double kp, double ki, double kd,
        double u_lim = 50.0,
        AntiWindup mode = AntiWindup::Conditional);
    void reset();
    double update(double target, double current, double dt);
};
}
```

### `arm/lowpass.hpp`

```cpp
namespace arm {
class LowPassFilter {
public:
    LowPassFilter(double tau, double dt);   // 时间常数、采样周期
    double update(double input);
    void reset();
    double alpha() const;                    // = dt/(tau+dt)
};
}
```

### `arm/kinematics.hpp`

```cpp
namespace arm {
// 标准型 D-H：T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
Eigen::Matrix4d dh(double theta, double d, double a, double alpha);

// 齐次变换闭式逆：T = [R p; 0 1]  ->  T^-1 = [R^T, -R^T*p; 0 1]
Eigen::Matrix4d fast_inverse(const Eigen::Matrix4d& T);
}
```

---

## 设计上的几个决定

- **`PID::update` 在 `dt <= 0` 时直接返回 `0.0`**，不做任何计算。
  否则微分项会除零得到 `inf`，再被限幅夹成一个"看起来合法"的满功率输出 —— 静默的错比崩溃更危险。
  守卫写成 `if (!(dt > 0.0))` 而不是 `if (dt <= 0.0)`，是为了挡住 `NaN`。
- **`LowPassFilter` 的构造传 `(tau, dt)` 而不是 `alpha`**。`alpha = dt/(tau+dt)` 是内部实现；
  调用方脑子里想的是「我要一个 0.1 秒的滤波器」，不是「我要 0.0909」。
- **`fast_inverse` 只对合法的齐次变换成立**（要求 `R` 是正交阵）。它比 `T.inverse()` 快一个数量级，误差也不累积。
