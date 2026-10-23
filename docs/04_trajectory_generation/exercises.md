# Phase 4 练习

1. 把 `compare_demo.cpp` 里的两段时间从 `1.5, 1.5` 改成 `3.0, 3.0`，重新编译并运行。记录最大速度、最大加速度和 jerk 代价。
2. 用单元测试里的接口，把单段终点速度从 0 改成 0.5 rad/s，看 `getVelocity(T)` 是否等于 0.5。
3. 用 `ros2 run` 不带参数运行 `compare_trajectories`，再用 `plot_trajectory.py` 看六条关节曲线是否同时结束。
