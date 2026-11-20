// PID 单元测试：只测「边界」，不测正常路径
//
// 判断一条测试有没有用的标准：
//   把实现改坏，它会不会红？
//   —— 不会红的测试不是"覆盖率"，是假的安全感。
//
// 所以下面每条断言都用了「恒定输入之外」的输入：
//   * 微分项必须靠"两拍之间 current 变化"才测得出来
//   * 抗饱和必须靠"饱和之后再反转"才测得出来
//   * 恒定输入下任何滤波器 / 任何阶次的控制器都会给出恒定输出，什么都证明不了
#include "arm/pid.hpp"

#include <cmath>
#include <cstdio>

static int g_fail = 0;

static void EXPECT_NEAR(double got, double want, double tol, const char* what) {
    if (std::fabs(got - want) > tol) {
        std::printf("[FAIL] %-40s got %.12f  want %.12f\n", what, got, want);
        ++g_fail;
    } else {
        std::printf("[ ok ] %-40s %.12f\n", what, got);
    }
}

static void EXPECT_TRUE(bool cond, const char* what) {
    if (!cond) { std::printf("[FAIL] %-40s\n", what); ++g_fail; }
    else       { std::printf("[ ok ] %-40s\n", what); }
}

int main() {
    std::printf("--- PID ---\n");

    // 1) 纯 P：u = kp * e
    {
        arm::PID pid(2.0, 0.0, 0.0, 50.0, arm::AntiWindup::None);
        EXPECT_NEAR(pid.update(10.0, 0.0, 0.01), 20.0, 1e-12, "1 纯P: u = kp*e");
    }

    // 2) 第一拍微分不冲：kd 再大，第一拍 de 也必须是 0
    {
        arm::PID pid(0.0, 0.0, 5.0, 50.0);
        EXPECT_NEAR(pid.update(10.0, 0.0, 0.01), 0.0, 1e-12, "2 第一拍微分不冲");
    }

    // 3) 微分先行：只对 current 求导（不是对误差求导）
    //    第 1 拍 current: 0 -> 0，de = 0
    //    第 2 拍 current: 0 -> 1，de = -(1-0)/dt = -100 -> u = kd*(-100)
    //    ⚠️ 这条是唯一能验出"微分项存不存在"的测试
    {
        arm::PID pid(0.0, 0.0, 0.1, 50.0, arm::AntiWindup::None);
        pid.update(0.0, 0.0, 0.01);
        EXPECT_NEAR(pid.update(0.0, 1.0, 0.01), -10.0, 1e-12, "3 微分先行: -(1-0)/dt*kd");
    }

    // 4) 限幅：正负两个方向都要夹
    {
        arm::PID pid(1000.0, 0.0, 0.0, 5.0, arm::AntiWindup::None);
        EXPECT_NEAR(pid.update(10.0, 0.0, 0.01), 5.0, 1e-12, "4a 限幅 +5");
        EXPECT_NEAR(pid.update(-10.0, 0.0, 0.01), -5.0, 1e-12, "4b 限幅 -5");
    }

    // 5) 条件积分：一直饱和时，积分器不该攒货
    //    kp=100 一开始就把输出打到 +10（饱和），于是 u != raw，条件积分拒绝累加
    //    饱和 1000 拍之后把目标变回 0，输出应该立刻松手（u = 0）
    {
        arm::PID pid(100.0, 100.0, 0.0, 10.0, arm::AntiWindup::Conditional);
        for (int i = 0; i < 1000; ++i) pid.update(1.0, 0.0, 0.01);
        EXPECT_NEAR(pid.update(0.0, 0.0, 0.01), 0.0, 1e-9, "5a 条件积分: 松手就到位");
    }

    // 6) 对照组：同样的场景，但关掉抗饱和 —— 积分器攒了 1000 拍的货，松不了手
    //    ⚠️ 这条和第 5 条必须给出不同的结果，否则第 5 条证明不了任何事
    {
        arm::PID pid(100.0, 100.0, 0.0, 10.0, arm::AntiWindup::None);
        for (int i = 0; i < 1000; ++i) pid.update(1.0, 0.0, 0.01);
        EXPECT_NEAR(pid.update(0.0, 0.0, 0.01), 10.0, 1e-9, "5b 无抗饱和: 还卡在 +10");
    }

    // 7) reset 清空积分和微分历史
    {
        arm::PID pid(1.0, 1.0, 0.0, 50.0, arm::AntiWindup::None);
        for (int i = 0; i < 100; ++i) pid.update(1.0, 0.0, 0.01);
        const double before = pid.update(0.0, 0.0, 0.01);
        EXPECT_TRUE(before > 0.5, "7a reset 前: 积分还在起作用");
        pid.reset();
        EXPECT_NEAR(pid.update(0.0, 0.0, 0.01), 0.0, 1e-12, "7b reset 后: 全部归零");
    }

    // ===== 以下是 Day 5 补的边界 =====

    // 8) dt = 0：除不起零？输出会不会被推到限幅？
    {
        arm::PID pid(1.0, 1.0, 1.0, 50.0, arm::AntiWindup::None);
        pid.update(1.0, 0.0, 0.01);                    // 先正常走一拍
        const double u = pid.update(1.0, 1.0, 0.0);    // 再来一拍 dt=0
        EXPECT_TRUE(std::fabs(u) < 50.0, "8 dt=0: 不该被推到限幅");
    }

    // 9) dt < 0：时间倒流
    {
        arm::PID pid(1.0, 1.0, 1.0, 50.0, arm::AntiWindup::None);
        pid.update(1.0, 0.0, 0.01);
        const double u = pid.update(1.0, 2.0, -0.01);
        EXPECT_TRUE(std::fabs(u) < 50.0, "9 dt<0: 不该被推到限幅");
    }

    // 10) ki = 0 且 Clamp 模式：钳位常量要算 u_lim/ki，会不会除零
    {
        arm::PID pid(1.0, 0.0, 0.0, 10.0, arm::AntiWindup::Clamp);
        const double u = pid.update(1.0, 0.0, 0.01);
        EXPECT_TRUE(std::isfinite(u), "10 ki=0 + Clamp: 不除零");
    }

    // 11) u_lim = 0：输出应该就是 0
    {
        arm::PID pid(1.0, 1.0, 0.0, 0.0, arm::AntiWindup::Conditional);
        EXPECT_NEAR(pid.update(1.0, 0.0, 0.01), 0.0, 1e-12, "11 u_lim=0: 输出 0");
    }

    // 12) 极端值：kp 和误差都取 1e9，看会不会溢出成 inf
    {
        arm::PID pid(1e9, 0.0, 0.0, 50.0, arm::AntiWindup::None);
        EXPECT_TRUE(std::isfinite(pid.update(1e9, 0.0, 0.01)), "12 kp=1e9 e=1e9: 有限");
    }

    std::printf("\n");
    if (g_fail == 0) { std::printf("All PID tests passed!\n"); return 0; }
    std::printf("PID: %d failed\n", g_fail);
    return 1;
}
