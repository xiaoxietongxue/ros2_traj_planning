# Master Development Plan

项目定位：面向初学者的机器人运动规划教学与工程实践平台。

开发模式：Tutorial-Driven Development。

主要技术：ROS 2 Humble（当前虚拟机）、MoveIt 2、C++17、Eigen、OMPL、MINCO。

## 最终功能目标

1. 搭建 ROS 2 C++ 开发环境。
2. 加载 UR5e 六自由度机械臂模型。
3. 使用 RViz 2 实现机器人可视化。
4. 使用 MoveIt 2 完成关节空间运动规划。
5. 使用 MoveIt 2 完成笛卡尔目标位姿规划。
6. 使用 OMPL 完成机械臂避障路径规划。
7. 实现正运动学、逆运动学和雅可比计算。
8. 实现五次多项式轨迹生成。
9. 实现多段 Minimum Jerk 轨迹优化。
10. 研究并移植 MINCO。
11. 将自定义轨迹生成器接入 MoveIt 2。
12. 使用 ros2_control 执行规划轨迹。
13. 建立轨迹性能评估系统。
14. 形成完整中文教程和实验案例。

## 阶段一览

| 阶段 | 内容 | 状态 |
|---|---|---|
| Phase 1 | ROS 2 环境、工作空间、C++ Topic 通信 | 已完成 |
| Phase 2 | UR5e 模型与 MoveIt 2 基础规划 | 已完成 |
| Phase 3 | 机械臂运动学（FK / IK / Jacobian） | 已完成 |
| Phase 4 | 五次多项式轨迹生成 | 已完成 |
| Phase 5 | Minimum Jerk 轨迹优化 | 已完成 |
| Phase 6 | MINCO 研究与移植 | 已完成 |
| Phase 7 | MoveIt 2 与轨迹优化集成 | 已完成 |
| Phase 8 | 轨迹性能评估与实验 | 已完成 |

## 架构原则

```
用户目标 → MoveIt 2 Planning Pipeline → OMPL 几何路径
        → 关节路径点 → 轨迹生成（Quintic / Minimum Jerk / MINCO）
        → 约束检查 → JointTrajectory → ros2_control → 评估
```

- MoveIt 2 是运动规划框架，OMPL 负责几何路径，轨迹模块负责带时间的连续轨迹，ros2_control 负责执行。
- 算法核心尽量独立于 ROS 2，ROS 接口单独做适配层。
- 不为目录结构创建空包。后续阶段按需新增功能包。

## 教程规范

每个 Phase 按以下顺序组织：学习目标 → 基础理论 → 数学原理 → 工程架构 → 接口设计 → 代码实现 → 关键代码讲解 → 编译运行 → 结果验证 → 常见问题 → 练习 → 阶段总结。

教程必须与仓库代码一致。性能数据必须来自实际运行。

## 执行规则（摘要）

1. 严格分阶段，首次只做 Phase 1。
2. 先讲解，再开发。
3. 边开发边教学。
4. 自动编译与调试，直到通过或确认外部阻碍。
5. 阶段结束必须生成教程和 `development_report.md`。
6. 不得伪造结果。
7. 未经确认不得删除已有 ROS 工作空间、不得更换发行版。
8. 阶段完成后停止，等待用户确认再进入下一 Phase。
9. 优先使用官方资料，记录第三方源码许可证。
10. 不要过度工程化。
