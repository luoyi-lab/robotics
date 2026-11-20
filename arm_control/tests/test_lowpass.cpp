// 一阶低通滤波器单元测试
//
// ⚠️ 关键：不要用「恒定输入」来测滤波器。
//    恒定输入 100 进来，正确的滤波器输出 100，坏掉的「导线」也输出 100，
//    alpha 从 0.001 到 0.999 全都输出 100 —— 这样的断言永远不可能红。
//
//    必须用【突变输入】（先 0 后 1），才能看到滤波器的"磨圆"效果。
#include "arm/lowpass.hpp"

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
    std::printf("--- LowPassFilter ---\n");

    // 1) alpha 由 (tau, dt) 推导：alpha = dt/(tau+dt)
    {
        arm::LowPassFilter f(0.1, 0.01);
        EXPECT_NEAR(f.alpha(), 0.01 / 0.11, 1e-15, "1 alpha = dt/(tau+dt)");
    }

    // 2) 第一拍直通：没有历史数据时，输入直接当输出
    //    （否则真实信号 100 会被"拉"到 alpha*100 ≈ 9）
    {
        arm::LowPassFilter f(0.1, 0.01);
        EXPECT_NEAR(f.update(100.0), 100.0, 1e-12, "2 第一拍直通");
    }

    // 3) 递归式第 1 步 —— 用闭式解 1-(1-a)^1 校验
    //    先喂 0 把 first_ 用掉，再喂 1：y = a*1 + (1-a)*0 = a
    //    ⚠️ 这条是唯一能验出"到底滤没滤"的测试：导线会给出 1.0，正确版给出 0.0909
    {
        arm::LowPassFilter f(0.1, 0.01);
        const double a = f.alpha();
        f.update(0.0);
        const double y1 = f.update(1.0);
        EXPECT_NEAR(y1, 1.0 - std::pow(1.0 - a, 1), 1e-12, "3 递归式第1步 = a");
        EXPECT_TRUE(y1 < 0.2, "3b 不是导线（突变被磨圆了）");
    }

    // 4) 递归式第 2 步 —— 闭式解 1-(1-a)^2
    {
        arm::LowPassFilter f(0.1, 0.01);
        const double a = f.alpha();
        f.update(0.0);
        f.update(1.0);
        const double y2 = f.update(1.0);
        EXPECT_NEAR(y2, 1.0 - std::pow(1.0 - a, 2), 1e-12, "4 递归式第2步");
    }

    // 5) 阶跃稳态 -> 1
    {
        arm::LowPassFilter f(0.1, 0.01);
        f.update(0.0);
        for (int k = 0; k < 5000; ++k) f.update(1.0);
        EXPECT_NEAR(f.update(1.0), 1.0, 1e-6, "5 阶跃稳态 -> 1");
    }

    // 6) t = 3*tau 时约到 95%（连续系统 1-e^-3 = 0.9502，离散略慢）
    //    dt=0.01, tau=0.1 -> 3*tau = 0.30s = 30 拍
    {
        arm::LowPassFilter f(0.1, 0.01);
        const double a = f.alpha();
        f.update(0.0);
        double y = 0.0;
        for (int k = 0; k < 30; ++k) y = f.update(1.0);
        EXPECT_NEAR(y, 1.0 - std::pow(1.0 - a, 30), 1e-12, "6 3*tau = 1-(1-a)^30");
        EXPECT_TRUE(std::fabs(y - 0.95) < 0.02, "6b 3*tau 附近约 0.95");
    }

    // 7) reset 之后第一拍又直通
    {
        arm::LowPassFilter f(0.1, 0.01);
        for (int k = 0; k < 100; ++k) f.update(1.0);
        const double before = f.update(0.0);
        EXPECT_TRUE(before > 0.9, "7a reset 前: 还记着历史");
        f.reset();
        EXPECT_NEAR(f.update(42.0), 42.0, 1e-12, "7b reset 后: 第一拍又直通");
    }

    // 8) alpha 越大越不滤波：alpha=0.9 时一步就冲到 90%
    {
        arm::LowPassFilter f(0.1, 0.9);   // alpha = 0.9/1.0 = 0.9
        f.update(0.0);
        EXPECT_TRUE(f.update(1.0) > 0.85, "8 alpha 大 -> 几乎不滤波");
    }

        // ===== 以下是 Day 5 补的边界 =====

    // 13) tau = 0 -> alpha = 1 -> 等效于一根导线（合法的极端值）
    {
        arm::LowPassFilter f(0.0, 0.01);
        EXPECT_TRUE(f.alpha() > 0.0 && f.alpha() <= 1.0, "13 tau=0: alpha 在 (0,1]");
    }

    // 14) tau + dt = 0 -> alpha = dt/0，会除零
    {
        arm::LowPassFilter f(-0.01, 0.01);
        EXPECT_TRUE(std::isfinite(f.alpha()), "14 tau+dt=0: alpha 有限");
    }

    // 15) tau < 0 -> alpha > 1 -> 反馈项 (1-alpha) 变负，输出会来回抽
    {
        arm::LowPassFilter f(-0.005, 0.01);
        EXPECT_TRUE(f.alpha() > 0.0 && f.alpha() <= 1.0, "15 tau<0: alpha 在 (0,1]");
    }

    // 16) 阶跃响应应该单调往上爬，不该回头
    {
        arm::LowPassFilter f(0.1, 0.01);
        f.update(0.0);
        bool mono = true;
        double prev = -1.0;
        for (int k = 0; k < 50; ++k) {
            const double y = f.update(1.0);
            if (y < prev) mono = false;
            prev = y;
        }
        EXPECT_TRUE(mono, "16 阶跃响应单调上升");
    }

    std::printf("\n");
    if (g_fail == 0) { std::printf("All LowPass tests passed!\n"); return 0; }
    std::printf("LowPass: %d failed\n", g_fail);
    return 1;
}