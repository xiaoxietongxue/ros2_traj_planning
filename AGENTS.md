# Agent 执行说明

本文件约束后续在本仓库中工作的编码代理。

## 项目性质

这不是一次性代码生成任务，而是长期的教学 + 工程仓库。必须同时保证：代码可运行、架构可维护、算法可验证、教程可学习、实验可复现。

## 必须遵守

1. **只做当前已确认的 Phase。** 未经用户确认不得进入下一阶段。
2. **先检查环境，再写代码。** 复用已有 ROS 2 Humble，不得覆盖发行版。
3. **Ubuntu 是 ARM64 虚拟机。** 安装依赖前先检查 aarch64 兼容性。可视化优先 RViz 2 + Mock Hardware，不要在早期引入 Gazebo。
4. **保护已有环境。** 不得删除 `/home/parallels/ros2_ws`、`catkin_ws` 或系统 Python。
5. **算法核心不依赖 ROS 2。** Quintic / Minimum Jerk / MINCO 用 C++17 + Eigen；ROS 只做适配层。
6. **不为未来阶段预建空包。**
7. **每个阶段结束必须：**
   - 编译通过
   - 实际运行验证
   - 更新对应 `docs/<phase>/`
   - 写 `development_report.md`
   - 汇报并等待确认
8. **不得伪造运行结果。** 跑不了的测试必须标明原因。
9. **Git：** 每个 Phase 完成后单独提交；未经确认不得改写历史。
10. **语言：** 教程、开发报告、对用户说明使用简体中文。

## 当前阶段

Phase 3：`src/robot_kinematics` 与 `docs/03_robot_kinematics`。未经确认不得进入 Phase 4。
