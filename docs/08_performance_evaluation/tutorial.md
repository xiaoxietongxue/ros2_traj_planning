# Phase 8：性能对比

## 学习目标

- 用同一组边界条件比较算法。
- 分开记录计算时间、轨迹时间、jerk、速度、加速度、连续性和碰撞。
- 知道 mock hardware 不能拿来当动力学实验。

完成标准：`results/phase8_metrics.csv` 和 `results/phase8_comparison.png` 由程序生成，表中的数和终端输出一致。

## 三个实验

1. `single_target`：两个关节路径点，2 秒，没有障碍。
2. `multi_waypoint`：四个路径点，每段 1.2 秒，总时间 3.6 秒，没有障碍。
3. `obstacle`：与实验 2 相同的轨迹，但在五次多项式 \(t=0.6\) 秒的末端位置放一个 5 cm 的盒子。

加速度检查阈值是 10 rad/s²。位置和速度用 URDF 限位。

## 运行

```bash
source /opt/ros/humble/setup.bash
source ~/robot_motion_ws/install/setup.bash
ros2 run robot_trajectory_execution experiment_runner
python3 scripts/analyze_performance.py results/phase8_metrics.csv results/phase8_comparison.png
```

## 实测表

| 实验 | 算法 | 计算 ms | 轨迹时间 s | jerk | 最大速度 | 最大加速度 | 连续 | 限位 | 无碰撞 |
|---|---|---:|---:|---:|---:|---:|---|---|---|
| single_target | quintic | 0.0012 | 2.000 | 34.200 | 0.7500 | 1.1547 | true | true | true |
| single_target | minimum_jerk | 0.0053 | 2.000 | 34.200 | 0.7500 | 1.1547 | true | true | true |
| single_target | minco | 0.0019 | 2.000 | 34.200 | 0.7500 | 1.1547 | true | true | true |
| multi_waypoint | quintic | 0.0007 | 3.600 | 575.810 | 0.7812 | 2.0037 | true | true | true |
| multi_waypoint | minimum_jerk | 0.0187 | 3.600 | 111.033 | 0.6985 | 1.4498 | true | true | true |
| multi_waypoint | minco | 0.1446 | 3.600 | 111.033 | 0.6985 | 1.4498 | true | true | true |
| obstacle | quintic | 0.0004 | 3.600 | 575.810 | 0.7812 | 2.0037 | true | true | false，26 个采样点 |
| obstacle | minimum_jerk | 0.0188 | 3.600 | 111.033 | 0.6985 | 1.4498 | true | true | false，34 个采样点 |
| obstacle | minco | 0.1335 | 3.600 | 111.033 | 0.6985 | 1.4498 | true | true | false，34 个采样点 |

盒子中心是 `(0.666189, 0.285619, 0.507742)` 米，坐标系 `world`。

## 怎么读这张表

- 只有一段时，三种算法是同一条曲线。
- 多个路径点时，不必在中间点停住，jerk 从 575.810 降到 111.033。
- 这组等分时间已经是 MINCO 的最优分配，所以它没有再降。
- 另一组两段轨迹（`compare_trajectories`）里，MINCO 把 jerk 从 28.945 降到 26.751。比较的时候要写明路径点是不是同一组。
- 障碍场景里三条都不允许执行。无碰撞的几何折线不能代替优化后的连续轨迹。
- 计算时间都在 1 毫秒以内到 0.15 毫秒这一量级。这是一次运行，不是统计均值。

Mock 执行只说明控制器接口接受了轨迹。它没有测量跟踪误差或关节力矩。

## 练习

见 [exercises.md](exercises.md)。

## 阶段总结

到这里，平台能从关节目标走到一条被检查过的时间轨迹，并能说明三种算法差在哪里。后续如果要把碰撞放进优化，需要新的约束，而不是把现在的事后检查改名。
