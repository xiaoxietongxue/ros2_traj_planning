# Phase 5 开发报告

日期：2026-10-23

## 已实现功能

- `MinimumJerkTrajectory`：固定分段时间，优化中间速度和加速度。
- 块三对角 Thomas 求解。
- 与五次多项式使用相同路径点、相同时间和相同起终点状态。

## 实测

两段各 1.5 秒：

```text
quintic jerk=127.052
minimum_jerk jerk=28.9449 continuous=true
```

四路径点、每段 1.2 秒（`experiment_runner`）：

```text
quintic jerk=575.810
minimum_jerk jerk=111.033
```

单段两者都是 34.200。

## 已知问题

时间仍然是输入，不是优化结果。加速度上限还没加进求解器，只在 Phase 7 事后检查。

## 下一阶段

MINCO 的时间分配。
