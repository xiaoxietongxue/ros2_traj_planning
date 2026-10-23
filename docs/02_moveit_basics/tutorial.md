# Phase 2：UR5e 模型与 MoveIt 2 基础

## 学习目标

- 看懂 URDF 里的 link / joint，以及 Xacro 为什么存在。
- 理解 TF2 如何表示机器人坐标系。
- 区分路径规划与轨迹规划。
- 用 MoveIt 2 做关节目标、末端位姿目标和笛卡尔路径规划。
- 往 PlanningScene 里加障碍物并观察规划结果。

前置：Phase 1 已能编译运行。本阶段在 **ROS 2 Humble + aarch64** 上复用官方包，没有换成 Jazzy。

完成标准：

1. `planning_demo` 在无界面模式下完成关节 / 位姿 / 笛卡尔 / 带障碍规划。
2. 你能在虚拟机桌面用 RViz 2 看到 UR5e，并用 MotionPlanning 插件拖目标。

## 基础理论

### URDF 和 Xacro

**URDF**（Unified Robot Description Format）用 XML 描述机器人：有哪些连杆（link）、关节（joint）、它们怎么连、外观网格在哪。

**Xacro** 是带宏和参数的 URDF 生成器。UR 系列很多机型共用同一套宏，通过 `ur_type:=ur5e` 选出尺寸和限位。

本项目不自己写 URDF，直接用官方包 `ur_description`。查看模型：

```bash
ros2 launch robot_bringup view_ur5e.launch.py
```

### Link 和 Joint

- **Link**：刚体，例如 `base_link`、`forearm_link`、`tool0`。
- **Joint**：两个 link 之间的运动约束。UR5e 有 6 个转动关节：
  `shoulder_pan`、`shoulder_lift`、`elbow`、`wrist_1`、`wrist_2`、`wrist_3`。

`tool0` 是法兰默认末端坐标系，MoveIt 里用它做末端。

### TF2

TF2 是 ROS 2 的坐标变换树。知道 `shoulder_pan_joint` 的角度后，`robot_state_publisher` 就能算出 `tool0` 相对 `base_link` 的位姿。

RViz 左上角 **Fixed Frame** 必须设成 `base_link`（或 `world`，如果树里有），否则模型会飞到空中或看不见。

### MoveIt 2 里的三个对象

| 名字 | 它是什么 | 输入 / 输出 |
|---|---|---|
| RobotModel | 从 URDF + SRDF 读出的机器人模型 | 文件 → 关节限位、规划组 |
| RobotState | 某一瞬间的关节角、末端位姿 | 关节角 ⇄ 笛卡尔位姿 |
| PlanningScene | 机器人 + 障碍物 + 碰撞模型 | 查询“这个状态碰不碰” |

你几乎不直接操作它们，而是通过 **MoveGroupInterface** 说话。MoveGroupInterface 是 C++ 客户端，背后的服务在 `move_group` 节点里。

### OMPL

OMPL（Open Motion Planning Library）做**几何路径规划**：在关节空间里找一条从起点到终点、不碰撞的折线。它不管电机加速度是否舒服。

MoveIt 默认再接一个时间参数化插件，把折线变成带时间戳的 `JointTrajectory`。那一步仍然很粗，所以后面才有 Quintic / MINCO。

### 路径规划 vs 轨迹规划

| | 路径 Path | 轨迹 Trajectory |
|---|---|---|
| 有没有时间 | 没有 | 有 |
| 典型输出 | 一串路点 | `q(t), qd(t), qdd(t)` |
| 本阶段谁负责 | OMPL | MoveIt 默认时间参数化 |
| 后面谁负责 | 仍然可以是 OMPL | Quintic / Minimum Jerk / MINCO |

## 工程架构

见 [architecture.md](architecture.md)。新增两个功能包：

- `robot_bringup`：启动文件
- `robot_motion_planning`：C++ 规划封装

官方包只作为依赖，不拷进仓库。

## 接口设计

```cpp
class MotionPlanner {
  PlanResult planToNamedTarget(const std::string & name);
  PlanResult planToJointTarget(const std::vector<double> & joints);
  PlanResult planToPoseTarget(const geometry_msgs::msg::Pose & pose);
  PlanResult planCartesianPath(const std::vector<geometry_msgs::msg::Pose> & waypoints);
  void addBoxObstacle(...);
};
```

规划组固定为 `ur_manipulator`，末端固定为 `tool0`。

`PlanResult` 包含是否成功、规划时间、轨迹点数和说明文字。失败不抛到 main 之外，由 demo 统计失败次数。

## 代码实现要点

### 为什么要单独开一个 spin 线程

`MoveGroupInterface` 内部要通过话题/服务和 `move_group` 通信。如果主线程卡在 `plan()` 上，又没有别的线程 `spin()`，回调到不了，规划会一直等。

```cpp
std::thread spinner([&executor]() { executor.spin(); });
```

### 关节目标

`setJointValueTarget` 按规划组关节顺序设置 6 个数，单位是弧度。顺序用 `jointNames()` 打印，不要凭感觉。

### 位姿目标

`setPoseTarget(pose, "tool0")` 后，MoveIt 先做 IK，再在关节空间规划。IK 失败时，看起来像“规划失败”，日志里会提到 IK。

### 笛卡尔路径

`computeCartesianPath` 在笛卡尔空间按步长插值，再逐点 IK。它**不是** OMPL。障碍物多、步长大时，`fraction` 会小于 1。

### 障碍物

`PlanningSceneInterface::applyCollisionObject` 把盒子加进场景。几何路径无碰，不代表以后自己重插值的连续轨迹一定无碰。本阶段只检查 OMPL / 笛卡尔插值本身。

## 编译运行

```bash
cd ~/robot_motion_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_motion_planning robot_bringup
source install/setup.bash
```

只看模型：

```bash
ros2 launch robot_bringup view_ur5e.launch.py
```

无界面跑规划验收（虚拟机里用 SSH / `prlctl` 也能跑）：

```bash
ros2 launch robot_bringup ur5e_moveit.launch.py launch_rviz:=false run_demo:=true
```

在 Parallels 桌面打开图形界面：

```bash
ros2 launch robot_bringup ur5e_moveit.launch.py launch_rviz:=true
```

RViz 中：

1. Fixed Frame = `base_link`
2. 左侧应有 MotionPlanning 插件
3. Planning Group = `ur_manipulator`
4. 拖橙色目标球，点 Plan

## 结果验证

无界面 demo 应打印类似：

```text
[named:up] success=true
[joint:side] success=true
[pose:offset] success=true
[cartesian] success=true
[pose:with_obstacle] success=true
[pose:into_box] success=false    # 预期：盒子内部目标被碰撞检测拒绝
Demo finished with 0 failed case(s)
```

进程退出码为 0。数字以你机器上的实际日志为准；OMPL 是随机采样规划器，时间和路径形状会变，成功与否应当稳定。

## 常见问题

见 [troubleshooting.md](troubleshooting.md)。

## 练习

见 [exercises.md](exercises.md)。

## 阶段总结

本阶段把“官方 UR5e 模型 + MoveIt 规划框架 + 自己的 C++ 调用层”接起来了。还没有自己写多项式，也没有往控制器发轨迹。下一阶段做运动学计算。
