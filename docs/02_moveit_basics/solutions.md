# Phase 2 练习参考答案

## 练习 1

```cpp
if (side.size() >= 1) {
  side[0] = -0.6;
}
```

```bash
colcon build --symlink-install --packages-select robot_motion_planning
source install/setup.bash
ros2 launch robot_bringup ur5e_moveit.launch.py launch_rviz:=false run_demo:=true
```

UR5e `shoulder_pan` 限位大约 ±2π，`-0.6` 在范围内。超出限位时 `setJointValueTarget` 会失败。

## 练习 2

合法名字见 `/opt/ros/humble/share/ur_moveit_config/config/ompl_planning.yaml` 的 `planner_configs:`。改完必须重新编译。RRTConnect 通常比 RRT* 快、路径更“折”；RRT* 更追求短路径，时间更长。记下三次 `time=`，不要只看一次。

## 练习 3

```cpp
box_pose.position.y = 0.15;
```

盒子挡在不同位置时，OMPL 找到的绕行会变。若把盒子放到末端必经的位置且体积很大，规划可能超时失败，这是正常的。
