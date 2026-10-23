# 教程目录

这套教程配合仓库代码使用，不要只看文档不跑程序。

## 前置知识

| 文档 | 内容 |
|---|---|
| [Linux 基础](00_prerequisites/linux_basics.md) | 终端、路径、权限、常用命令 |
| [C++ 基础](00_prerequisites/cpp_basics.md) | 本项目会用到的 C++ 概念 |
| [ROS 2 基础](00_prerequisites/ros2_basics.md) | Node、Topic、工作空间 |

## 分阶段教程

| 阶段 | 文档 | 完成标准 |
|---|---|---|
| Phase 1 | [环境与 ROS 2 基础](01_environment_setup/tutorial.md) | 能编译并看到 talker / listener 通信 |
| Phase 2 | [UR5e 与 MoveIt 2 基础](02_moveit_basics/tutorial.md) | 已完成 |
| Phase 3 | [机械臂运动学](03_robot_kinematics/tutorial.md) | 已完成 |
| Phase 4 | [五次多项式](04_trajectory_generation/tutorial.md) | 已完成 |
| Phase 5 | [Minimum Jerk](05_minimum_jerk/tutorial.md) | 已完成 |
| Phase 6 | [MINCO](06_minco/tutorial.md) | 已完成 |
| Phase 7 | [系统集成](07_system_integration/tutorial.md) | 已完成 |
| Phase 8 | [性能评估](08_performance_evaluation/tutorial.md) | 已完成 |

Phase 1 附加文件：常见问题 / 练习 / 答案 / [开发报告](01_environment_setup/development_report.md)

Phase 2 附加文件：

- [架构](02_moveit_basics/architecture.md)
- [常见问题](02_moveit_basics/troubleshooting.md)
- [练习](02_moveit_basics/exercises.md)
- [参考答案](02_moveit_basics/solutions.md)
- [开发报告](02_moveit_basics/development_report.md)

Phase 3 附加文件：

- [数学](03_robot_kinematics/mathematics.md)
- [练习](03_robot_kinematics/exercises.md)
- [参考答案](03_robot_kinematics/solutions.md)
- [开发报告](03_robot_kinematics/development_report.md)

Phase 4–8 的教程、练习和开发报告在对应目录：

- `04_trajectory_generation`
- `05_minimum_jerk`
- `06_minco`（含 [论文笔记](06_minco/paper_notes.md)）
- `07_system_integration`
- `08_performance_evaluation`
