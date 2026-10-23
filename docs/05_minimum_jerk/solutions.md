# Phase 5 参考答案

1. 路径点越多，五次多项式要停的次数越多，jerk 代价通常升得更快。`experiment_runner` 的 `multi_waypoint` 有 4 个路径点、总时间 3.6 秒：五次多项式 jerk 为 575.810，minimum jerk 为 111.033。
2. 时间分配会改变代价。`compare_trajectories` 里 MINCO 把 `1.5, 1.5` 改成约 `1.350, 1.650` 后，minimum jerk 的 28.945 进一步降到 26.751。你手改成 `0.8, 2.2` 之后必须重新运行才能记下新数。
3. `single_target` 只有一段，三种算法 jerk 都是 34.200。
