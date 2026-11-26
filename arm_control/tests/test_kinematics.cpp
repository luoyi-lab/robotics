#include "arm/kinematics.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>

static int g_fail = 0;

static void EXPECT_NEAR(double got, double want, double tol, const char* what) {
    if (std::fabs(got - want) > tol) {
        std::printf("[FAIL] %-42s got %.12f want %.12f\n", what, got, want);
        ++g_fail;
    } else {
        std::printf("[ ok ] %-42s %.12f\n", what, got);
    }
}

static void EXPECT_TRUE(bool c, const char* what) {
    if (!c) { std::printf("[FAIL] %-42s\n", what); ++g_fail; }
    else    { std::printf("[ ok ] %-42s\n", what); }
}

int main() {
    std::printf("--- kinematics ---\n");

    // 0) 先验「这确实是个合法齐次变换」—— 最后一行必须是 [0 0 0 1]
    {
        const Eigen::Matrix4d T = arm::fk2link(0.3, 0.7, 1.0, 1.0);
        EXPECT_TRUE(T(3, 3) == 1.0, "0a 齐次矩阵右下角 = 1");
        EXPECT_TRUE(T.row(3).head<3>().norm() == 0.0, "0b 最后一行前三项 = 0");
    }

    // 1) 两连杆正运动学
    const Eigen::Matrix4d T = arm::dh(0.3, 0.0, 1.0, 0.0) * arm::dh(0.7, 0.0, 1.0, 0.0);
    EXPECT_NEAR(T(0, 3), 1.495639, 1e-6, "1a 末端 x");
    EXPECT_NEAR(T(1, 3), 1.136991, 1e-6, "1b 末端 y");
    EXPECT_NEAR(T(2, 3), 0.0,      1e-12, "1c 末端 z");
    EXPECT_NEAR(std::atan2(T(1, 0), T(0, 0)), 1.0, 1e-12, "1d 末端角度");

    // 2) 负角度
    {
        const Eigen::Matrix4d A = arm::dh(-0.3, 0.0, 1.0, 0.0);
        EXPECT_NEAR(A(0, 0),  0.955336, 1e-6, "2a cos(-0.3)");
        EXPECT_NEAR(A(1, 0), -0.295520, 1e-6, "2b sin(-0.3) 是负的");
    }

    // 3) 3D: alpha = pi/2
    {
        const Eigen::Matrix4d B = arm::dh(0.5, 0.2, 0.8, M_PI / 2);
        EXPECT_NEAR(B(0, 0), 0.877583, 1e-6, "3a 3D: R00");
        EXPECT_NEAR(B(0, 2), 0.479426, 1e-6, "3b 3D: R02 = sin(th)sin(al)");
        EXPECT_NEAR(B(1, 2), -0.877583, 1e-6, "3c 3D: R12");
        EXPECT_NEAR(B(2, 3), 0.2,      1e-12, "3d 3D: z 平移 = d");
    }

    // 4) 闭式逆: T * T^-1 = I
    {
        const Eigen::Matrix4d Ti = arm::fast_inverse(T);
        EXPECT_NEAR((T * Ti - Eigen::Matrix4d::Identity()).norm(), 0.0, 1e-12, "4a T * T^-1 = I");
    }

    // 5) 闭式逆和通用逆一致
    {
        const Eigen::Matrix4d Ti = arm::fast_inverse(T);
        EXPECT_NEAR((Ti - T.inverse()).norm(), 0.0, 1e-9, "5 闭式逆 = 通用逆");
    }

    // 6) 二次求逆回到原矩阵
    {
        const Eigen::Matrix4d Ti = arm::fast_inverse(T);
        EXPECT_NEAR((arm::fast_inverse(Ti) - T).norm(), 0.0, 1e-9, "6 inv(inv(T)) = T");
    }

    // 7) 全零参数 = 单位阵
    {
        const Eigen::Matrix4d I0 = arm::dh(0.0, 0.0, 0.0, 0.0);
        EXPECT_NEAR((I0 - Eigen::Matrix4d::Identity()).norm(), 0.0, 1e-15, "7 dh(0,0,0,0) = I");
    }
    
    // ===== fk2link（两连杆正运动学）=====
    // 8) 老熟人：0.3 / 0.7，等长连杆
    {
        const Eigen::Matrix4d T = arm::fk2link(0.3, 0.7, 1.0, 1.0);
        EXPECT_NEAR(T(0, 3), 1.495639, 1e-6, "8a fk2link 末端 x");
        EXPECT_NEAR(T(1, 3), 1.136991, 1e-6, "8b fk2link 末端 y");
        EXPECT_NEAR(T(2, 3), 0.0,      1e-12, "8c fk2link 末端 z");
        EXPECT_NEAR(std::atan2(T(1, 0), T(0, 0)), 1.0, 1e-12, "8d 末端角度 = θ1+θ2");
    }

    // 9) 全零：伸直，末端在 (L1+L2, 0)
    {
        const Eigen::Matrix4d T = arm::fk2link(0.0, 0.0, 1.0, 1.0);
        EXPECT_NEAR(T(0, 3), 2.0, 1e-12, "9a 伸直 x = L1+L2");
        EXPECT_NEAR(T(1, 3), 0.0, 1e-12, "9b 伸直 y = 0");
    }

    // 10) 竖直：θ1 = π/2, θ2 = 0 → 末端在 (0, 2)
    {
        const Eigen::Matrix4d T = arm::fk2link(M_PI / 2.0, 0.0, 1.0, 1.0);
        EXPECT_NEAR(T(0, 3), 0.0, 1e-12, "10a 竖直 x = 0");
        EXPECT_NEAR(T(1, 3), 2.0, 1e-12, "10b 竖直 y = L1+L2");
    }

    // 11) 不等长连杆 + 非特殊角
    {
        const Eigen::Matrix4d T = arm::fk2link(0.3, 0.7, 2.0, 0.5);
        EXPECT_NEAR(T(0, 3), 2.180824, 1e-6, "11a L1=2 L2=0.5 末端 x");
        EXPECT_NEAR(T(1, 3), 1.011776, 1e-6, "11b L1=2 L2=0.5 末端 y");
    }

    // 12) 折返奇异位形：θ2 = π 时末端回到原点（L1 = L2 才成立）
    {
        const Eigen::Matrix4d T = arm::fk2link(0.0, M_PI, 1.0, 1.0);
        EXPECT_NEAR(T(0, 3), 0.0, 1e-12, "12a 折返时 x = 0");
        EXPECT_NEAR(T(1, 3), 0.0, 1e-12, "12b 折返时 y = 0");
    }

    // 13) 和 dh() 拼出来的结果必须完全一致（交叉验证）
    {
        const Eigen::Matrix4d A = arm::fk2link(0.3, 0.7, 1.0, 1.0);
        const Eigen::Matrix4d B = arm::dh(0.3, 0.0, 1.0, 0.0) * arm::dh(0.7, 0.0, 1.0, 0.0);
        EXPECT_NEAR((A - B).norm(), 0.0, 1e-15, "13 fk2link == dh1*dh2");
    }

    std::printf("\n");
    if (g_fail == 0) { std::printf("All kinematics tests passed!\n"); return 0; }
    std::printf("kinematics: %d failed\n", g_fail);
    return 1;
}