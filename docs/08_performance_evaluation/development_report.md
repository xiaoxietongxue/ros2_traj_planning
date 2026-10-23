# Phase 8 开发报告

日期：2026-10-23

## 已实现功能

- `experiment_runner` 跑三个场景、三种算法。
- CSV：`results/phase8_metrics.csv`
- 图：`results/phase8_comparison.png`，以及三条轨迹曲线图。
- 脚本：`scripts/plot_trajectory.py`、`scripts/analyze_performance.py`

## 实测摘要

多路径点：五次多项式 jerk 575.810，minimum jerk 与 MINCO 都是 111.033。  
障碍：三者 `collision_free=false`。五次多项式 26 个碰撞采样，另外两种各 34 个。  
单目标：三者 jerk 都是 34.200，并且无碰撞。

这些数来自 2026-10-23 的一次 `experiment_runner` 输出。计算时间不是多次平均。

## 已知问题

加速度阈值不是厂商数据。Mock 执行没有跟踪误差。MINCO 在这组等分时间上没有进一步降代价；另一组路径上有下降，写在 Phase 6 报告里。

## 项目状态

Phase 1 到 Phase 8 的代码、测试和这次运行记录都在仓库里。碰撞约束还没有进入优化器。
