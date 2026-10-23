# Phase 6 开发报告

日期：2026-10-23

## 已实现功能

- `MincoTrajectory::generateFixedTime` 与 minimum jerk 对齐。
- `optimizeTimeAllocation`：总时间固定，softmax 参数化，梯度下降。
- 时间梯度用包络定理，单元测试用有限差分核对到 \(10^{-3}\)。
- 论文笔记和 MIT 许可证记录：`paper_notes.md`。没有把 GCOPTER 源码拷进仓库。

## 实测

```text
times=1.34974 1.65026
minimum_jerk jerk=28.9449
minco jerk=26.7509
```

四路径点等分实验中两者都是 111.0327。MoveIt 路径上 minco jerk=1.06669，低于 minimum jerk 的 2.48076。

## 已知问题

没有把障碍物放进优化。总时间必须由调用者给定。

## 下一阶段

把这三条轨迹接到 MoveIt 路径和 ros2_control。
