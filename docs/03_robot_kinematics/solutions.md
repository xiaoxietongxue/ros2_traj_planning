# Phase 3 练习参考答案

这些数来自 2026-10-23 在本机 Ubuntu 22.04 / ROS 2 Humble / UR5e 官方模型上的运行。KDL 对同一组种子是确定的；换种子后关节解可以不同，但位姿误差仍应很小。

## 1. 转动 shoulder_pan

在 `kinematics_demo.cpp` 里把

```cpp
regular << 0.3, -1.0, 1.2, -1.5, 1.2, 0.5;
```

改成第一项 `0.7`，然后：

```bash
cd ~/robot_motion_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_kinematics
source install/setup.bash
ros2 run robot_kinematics kinematics_demo
```

对照未修改时的位置 `0.6520 0.3790 0.5050`。新的 x、y 会变，z 变化较小。单元测试里也有同类检查：只给第 0 个关节加 `0.4` rad，末端位移大于 1 mm。

做完练习后把源码改回去，否则后面的验收数字对不上。

## 2. 可操作度

默认 demo 已经打印：

```text
manipulability regular / singular: 0.102818 / 0.000000
```

普通构型远离阈值 `1e-3`。`wrist_2 = 0` 的构型被标成 `near singularity`。

## 3. 靠近奇异

把普通构型最后第二个关节（`wrist_2`）从 `1.2` 改成 `0.05`，重新编译运行。可操作度应低于 `0.102818`。再改成 `0`，应接近 demo 里奇异构型的数量级。

若程序报 `outside [...]`，说明这个角度越出了带安全余量的 URDF 限位，换一个限位内的值。
