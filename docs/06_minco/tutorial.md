# Phase 6：MINCO

## 学习目标

- 说明 MINCO 和普通五次多项式不是一回事。
- 看懂“系数由线性系统恢复，优化变量是时间”。
- 在总时间不变时比较时间分配前后的 jerk 代价。

完成标准：固定时间的 MINCO 与 minimum jerk 代价一致；允许重新分配时间时，代价不高于等时间的 minimum jerk。

## 基础理论

见 [paper_notes.md](paper_notes.md)。这里只保留机械臂用得到的部分。

\(s=3\) 时，每一段仍是五次多项式，但中间速度不手写。时间 \(T_i\) 是优化变量。总时间固定，避免优化器把时间拉到无穷长、让 jerk 变成 0。

## 接口

```cpp
MincoTrajectory::generateFixedTime(...);      // 应与 minimum jerk 相同
MincoTrajectory::optimizeTimeAllocation(...); // 总时间固定
```

## 结果验证

`compare_trajectories`，两段，总时间 3 秒：

```text
minco_iterations=10 times=1.34974 1.65026
minimum_jerk jerk=28.9449
minco jerk=26.7509
```

等分时间不是最优。MINCO 把第一段缩短、第二段加长，代价从 28.945 降到 26.751。

`experiment_runner` 的四路径点、每段 1.2 秒：

```text
minimum_jerk jerk=111.0327
minco jerk=111.0327
```

这组路径上，等分已经是总时间 3.6 秒里的最优分配，所以代价没有再降。两种结果都来自实际运行。

MoveIt 抽出的 6 个路径点上，差别更明显：

```text
quintic jerk=806.072
minimum_jerk jerk=2.48076
minco jerk=1.06669
```

## 常见问题

1. 不要把 Phase 4 的插值叫成 MINCO。
2. 只有一段时，时间没有可分配的自由度，三者代价相同。
3. 代价下降不表示一定无碰撞。障碍实验里三条轨迹都撞上了盒子。

## 练习

见 [exercises.md](exercises.md)。

## 阶段总结

MINCO 在本项目里是“minimum jerk 系数恢复 + 固定总时间的时间分配”。几何避障仍留给生成后的检查。
