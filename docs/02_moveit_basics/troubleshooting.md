# Phase 2 常见问题

## `Package 'ur_description' not found`

没有 source 系统 ROS，或 MoveIt / UR 包没装上。

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ur_description
ros2 pkg prefix ur_moveit_config
```

本阶段安装的是 Humble 的官方 debian 包，不要改成 Jazzy。

## RViz 里看不到机器人

1. Fixed Frame 改成 `base_link`
2. 确认 `/robot_description` 有数据：`ros2 topic echo /robot_description --once`（很长，能刷出来即可）
3. 确认 `/joint_states` 在更新：`ros2 topic hz /joint_states`
4. Add → RobotModel，Description Topic 选 `/robot_description`

## `planning_demo` 卡在启动或抛异常

`move_group` 还没起来。demo 已经 sleep 4 秒；若机器很慢，把 `planning_demo.cpp` 里的等待加长，或先单独启动 launch，再另开终端：

```bash
ros2 run robot_motion_planning planning_demo
```

该终端也必须 `source install/setup.bash`。

## 位姿规划失败

常见原因：

- 目标在工作空间外
- 靠近肩部奇异（末端太靠近基座轴线）
- 姿态四元数不是单位四元数

先用 `currentPose` 附近的小偏移，不要随便填 `(1, 1, 1)`。

## 笛卡尔路径 `fraction < 1`

直线穿过关节限位或奇异。减小步长，或换一条不穿过基座附近的直线。这和 Phase 1 教程里 MINCO 笛卡尔路径“向内弯进奇异区”是同一类几何问题。

## `Unable to identify any set of controllers`

本阶段没有启动 `ros2_control`。**Plan 可以成功，Execute 会失败。** 这是预期现象，执行放到 Phase 7。RViz 里只点 Plan，不要点 Execute。

## 虚拟机里 RViz 很卡

降低 Displays 里 Grid / 阴影，关掉不需要的插件。规划本身可以 `launch_rviz:=false` 在终端验证。

## 想换成本地自己的 URDF

把 `ur5e_moveit.launch.py` 里的 `description` 指到你的 xacro 即可。规划组名、末端名如果变了，要同步改 `MotionPlanner` 构造参数和 SRDF。现在不要改官方 SRDF。
