# Phase 4 开发报告

日期：2026-10-23

## 已实现功能

- `QuinticTrajectory`：单关节五次多项式，满足两端位置、速度、加速度。
- `JointQuinticTrajectory`：多关节同步，中间路径点速度和加速度为 0。
- 采样、连续性检查、CSV。
- 单元测试和 `compare_trajectories`。

## 编译与测试

```text
Finished <<< robot_trajectory_optimization [3.18s]
[  PASSED  ] 6 tests.
```

其中五次多项式相关测试全部包含在这 6 个里面。

## 实测

总时间 3 秒、两段、六关节：

```text
quintic compute_ms=0.001209 duration_s=3 jerk=127.052
max_vel=0.625 max_acc=1.28291 max_jerk=8.88889 continuous=true
```

图：`results/phase4_quintic.png`（由脚本从 CSV 生成）。

## 已知问题

中间路径点被要求停住。这会抬高 jerk。Phase 5 去掉这个约束。

## 下一阶段

固定时间的 minimum jerk。
