# Phase 2 开发报告

日期：2026-10-23  
工作空间：`/home/parallels/robot_motion_ws`

## 已实现功能

- 从 apt 安装 Humble 官方包：MoveIt 2.5.10、`ur_description` 2.14.0、`ur_moveit_config` 2.15.0、OMPL、ros2_control 相关依赖。未装 Gazebo，未覆盖已有发行版。
- `robot_bringup`
  - `view_ur5e.launch.py`：复用官方 `view_ur.launch.xml`，只看模型
  - `ur5e_moveit.launch.py`：`robot_state_publisher` + `joint_state_publisher` + 官方 `ur_moveit.launch.py` + 可选 `planning_demo`
- `robot_motion_planning`
  - `MotionPlanner`：`planToNamedTarget` / `planToJointTarget` / `planToPoseTarget` / `planCartesianPath` / `addBoxObstacle`
  - `planning_demo`：无界面验收

没有自写 URDF，没有复制官方 MoveIt 配置。

## 主要文件

| 路径 | 作用 |
|---|---|
| `src/robot_bringup/launch/ur5e_moveit.launch.py` | 启动入口 |
| `src/robot_motion_planning/include/robot_motion_planning/motion_planner.hpp` | 规划 API |
| `src/robot_motion_planning/src/planning_demo.cpp` | 验收节点 |
| `docs/02_moveit_basics/tutorial.md` | 教程 |

## 核心技术

MoveGroupInterface + OMPL RRTConnect + KDL IK + PlanningScene 碰撞对象。本阶段不执行轨迹。

## 编译结果

```text
Finished <<< robot_motion_planning [3.18s]
Finished <<< robot_bringup [0.04s]
Summary: 2 packages finished [3.34s]
```

## 测试结果（无界面，实际运行）

```text
Current joints: 0.000 -1.571 0.000 0.000 0.000 0.000
Current EE pose: [0.100, 0.233, 0.980]
[named:up]            success=true  time=0.019s waypoints=58
[joint:side]          success=true  time=0.018s waypoints=21
[pose:offset]         success=true  time=10.001s waypoints=163
[cartesian]           success=true  waypoints=30
[pose:with_obstacle]  success=true  time=0.042s waypoints=158
[pose:into_box]       success=false   # 预期失败，碰撞检测拒掉盒子内部目标
Demo finished with 0 failed case(s)
```

RViz 图形界面未在 `prlctl` 无 DISPLAY 环境下打开。请在虚拟机桌面执行：

```bash
ros2 launch robot_bringup ur5e_moveit.launch.py launch_rviz:=true
```

Fixed Frame 选 `base_link`，Planning Group 选 `ur_manipulator`，只点 Plan，不要点 Execute。

## 已知问题

1. 官方 `ur_moveit.launch.py` 不发布 `/joint_states`。必须由 `robot_bringup` 额外启动 `joint_state_publisher`。参数文件必须带 `ros__parameters`，否则 JSP 启动失败。
2. 初始构型若用 SRDF 的 `up`（wrist_1 = -90°），末端约在 z=1.08 m，靠近 UR5e 伸展极限，笛卡尔规划会失败。现改为 `home`。
3. 规划坐标系是 `world`（URDF 里 world→base_link）。RViz Fixed Frame 用 `base_link` 即可。
4. 本阶段没有控制器，RViz 里 Execute 会失败，这是预期现象。
5. OMPL 是随机规划器，同一目标的时间和路点数会波动。

## 下一阶段预告（未开始）

Phase 3：基于 RobotModel / RobotState 做 FK、IK、雅可比和可操作度。  
**等待确认后再开始。**
