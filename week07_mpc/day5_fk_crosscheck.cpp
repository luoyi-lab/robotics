#include "arm/kinematics.hpp"      // ← 用库，不是把代码复制过来

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

int main(int argc, char** argv) {
    const char* path = (argc > 1) ? argv[1] : "fk_cases.csv";
    std::ifstream in(path);
    if (!in) { std::printf("打不开文件: %s\n", path); return 1; }

    std::string line;
    std::getline(in, line);              // 跳过表头

    int n = 0;
    double worst = 0.0;
    int worst_idx = -1;
    int bad = 0;                         // 超过阈值的组数

    while (std::getline(in, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string cell;
        std::vector<double> v;
        while (std::getline(ss, cell, ',')) v.push_back(std::stod(cell));
        if (v.size() != 6) { std::printf("第 %d 行列数不对\n", n); return 1; }

        const double t1 = v[0], t2 = v[1], L1 = v[2], L2 = v[3];
        const double px = v[4], py = v[5];

        const Eigen::Matrix4d T = arm::fk2link(t1, t2, L1, L2);
        const double dx = std::fabs(T(0, 3) - px);
        const double dy = std::fabs(T(1, 3) - py);
        const double dz = std::fabs(T(2, 3) - 0.0);      // 平面臂：z 必须是 0
        const double e  = std::max({dx, dy, dz});

        if (e > 1e-9) ++bad;
        if (e > worst) { worst = e; worst_idx = n; }
        ++n;
    }

    std::printf("比对了 %d 组\n", n);
    std::printf("最大误差 = %.3e（第 %d 组）\n", worst, worst_idx);
    std::printf("超过 1e-9 的组数 = %d\n", bad);

    if (bad == 0 && n == 50) { std::printf("对账通过 ✓\n"); return 0; }
    std::printf("对账失败 ✗\n");
    return 1;
}