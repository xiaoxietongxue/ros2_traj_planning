# Phase 4：五次多项式轨迹

## 学习目标

- 区分路径和轨迹。
- 会写五次多项式的位置、速度、加速度和 jerk。
- 用两端的位置、速度、加速度求出六个系数。
- 让六个关节共用同一时间轴。
- 把采样结果写成 CSV，并画出四条曲线。

前置：Phase 3 能计算 UR5e 的关节角和末端位姿。本阶段的算法不依赖 ROS 2。

完成标准：`test_trajectories` 通过，并且 `compare_trajectories` 写出 `results/phase4_quintic.csv`。

## 基础理论

路径是一串关节角 \(q_0,q_1,\ldots\)。轨迹是函数 \(q(t)\)。控制器要的是后者，因为它需要每个时刻的位置，最好还有速度。

三次多项式只能满足位置和速度四个条件。要同时指定两端的加速度，需要六个系数，所以用五次多项式。

## 工程架构

功能包 `robot_trajectory_optimization`，核心类是 `QuinticTrajectory` 和 `JointQuinticTrajectory`。它们只使用 C++17 和 Eigen。

多段轨迹在中间路径点把速度和加速度设为 0。六个关节的时间分段相同。

## 接口

```cpp
bool generate(const ScalarState & start, const ScalarState & end, double duration, std::string & error);
double getPosition(double t) const;
double getVelocity(double t) const;
double getAcceleration(double t) const;
double getJerk(double t) const;
```

`duration <= 0` 或状态里有 NaN 时，`generate` 返回 false。

## 编译运行

```bash
cd ~/robot_motion_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_trajectory_optimization
source install/setup.bash
colcon test --packages-select robot_trajectory_optimization --event-handlers console_direct+
ros2 run robot_trajectory_optimization compare_trajectories
python3 scripts/plot_trajectory.py results/phase4_quintic.csv results/phase4_quintic.png
```

`colcon build --packages-select` 只编译这一个包。成功时摘要是 `1 package finished`。

## 结果验证

这次比较用的三个关节路径点是

```text
(0.3, -1.0, 1.2, -1.5, 1.2, 0.5)
(0.0, -1.2, 1.4, -1.3, 0.9, 0.2)
(-0.4, -0.8, 0.9, -1.0, 1.1, -0.3)
```

每段 1.5 秒，总时间 3 秒，起终点速度和加速度为 0。

五次多项式实测：

```text
compute_ms=0.001209
jerk=127.052
max_vel=0.625
max_acc=1.28291
max_jerk=8.88889
continuous=true
```

连续性阈值是位置、速度、加速度跳变都小于 \(10^{-5}\)。

## 常见问题

1. 中间点速度为 0 是本阶段的选择，不是五次多项式的唯一用法。单段可以指定非零边界速度。
2. 时间越短，同样的位移需要更大的速度和 jerk。
3. 图在 `results/`，该目录里的 csv 和 png 不进 Git。需要自己再跑一遍命令生成。

## 练习

见 [exercises.md](exercises.md)。公式在 [mathematics.md](mathematics.md)。

## 阶段总结

五次多项式把两个关节状态连成一条带时间的曲线。多个路径点如果都要求停住，动作会一顿一顿。下一阶段去掉这个限制，只保留连续，并让 jerk 的能量最小。
