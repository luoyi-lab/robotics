# robotics

具身智能 / 运动控制方向的学习与项目仓库。

从零开始的 20 周执行记录：机器人学基础 → MPC → WBC → ROS 2 → 完整项目。
完整计划、里程碑、每周进度见 **[PLAN.md](PLAN.md)**。

## 目录

| 目录 | 内容 |
|---|---|
| `pybullet_intro/` | PyBullet 入门：GUI 交互、无界面渲染、GIF 导出 |
| `week01_kinematics/` | 旋转矩阵 / 欧拉角 / 四元数 / 齐次变换 |

## 环境

Ubuntu 26.04 (WSL 2) + Python 3.11 + PyBullet 3.2.7 + NumPy + Matplotlib。
一键搭建脚本见 [setup_env.sh](setup_env.sh)。

> pybullet 官方 wheel 只发布到 Python 3.11，而系统 Python 是 3.14，
> 所以用 `uv` 单独装了一个 3.11 解释器，虚拟环境在 `~/pybullet-env`。
> 跑任何脚本都用 `~/pybullet-env/bin/python`。

## 说明

本仓库记录的是**从传统机械方向转向运动控制**的完整过程，
包含每天的代码、笔记和踩坑记录。没有真机，全部工作在仿真中完成，
并会专门记录 Sim2Real 相关的分析。
