# Phase 5：Minimum Jerk

## 学习目标

- 分清优化变量、目标函数和等式约束。
- 理解固定时间优化：时间是给定的，不参与优化。
- 比较“路径点停住”和“只要求连续”的 jerk 代价。

完成标准：同一路径点和同一分段时间下，minimum jerk 的代价不高于五次多项式，并且速度、加速度连续。

## 基础理论

轨迹优化先选定变量。这里的变量不是多项式的全部系数，而是中间路径点的速度和加速度。系数由每一段的六个边界条件唯一确定。

目标是 jerk 平方的积分。它惩罚“加速度变化太猛”，所以轨迹更顺，但不保证速度或加速度不超过电机限制。限制要在生成之后检查，Phase 7 才做这件事。

## 接口

```cpp
MinimumJerkTrajectory::generate(waypoints, segment_durations, start_v, start_a, end_v, end_a, error);
```

`segment_durations` 的长度是路径点数减 1。时间必须为正。

## 结果验证

`compare_trajectories` 的实测：

```text
minimum_jerk compute_ms=0.00959 duration_s=3 jerk=28.9449
max_vel=0.541303 max_acc=1.04388 max_jerk=4.81481 continuous=true
```

对照五次多项式的 jerk 代价 127.052。边界条件、路径点和时间都相同。

单段情形下两者 jerk 都是 34.2（实验 `single_target`，总时间 2 秒）。这是预期结果，不是实现错误。

## 常见问题

1. 代价更小不等于一定更好执行。还要看速度、加速度和碰撞。
2. 时间分配仍然是手写的。改时间会改变代价，但那一步要到 MINCO 才自动做。

## 练习

见 [exercises.md](exercises.md)。

## 阶段总结

固定时间的 minimum jerk 让机械臂不必在每个路径点停住。下一段把分段时间也变成优化变量，那才是 MINCO 多出来的部分。
