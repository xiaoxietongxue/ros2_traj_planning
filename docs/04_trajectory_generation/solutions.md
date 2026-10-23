# Phase 4 参考答案

1. 时间加长以后，同样的位移被拉得更慢，最大速度、加速度和 jerk 代价都会下降。仓库里默认 1.5 秒一段的实测最大速度是 0.625 rad/s，jerk 代价是 127.052。改成 3 秒一段后需要你重新运行，不要沿用这个数。
2. `Quintic.RejectsBadDurationAndMatchesNonzeroBoundary` 已经检查了非零边界：终点速度 -0.3 时，`getVelocity(T)` 回到 -0.3。
3. 六条曲线的时间轴都是 0 到 3 秒。同步指的是共用时间，不是六条位置曲线形状相同。
