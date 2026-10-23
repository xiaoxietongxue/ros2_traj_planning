# Phase 7 练习

1. 把 `retime_moveit_path` 里的 `current[0] += 0.6` 改成 `0.3`，重新规划，记录路点数和三种 jerk。
2. 把采样周期从 0.02 改成 0.05，看 `joint_trajectory_points` 是否变少。
3. 在障碍场景里确认碰撞为 false 的轨迹不会被 `execute_trajectory` 使用。当前障碍实验三条都是碰撞，所以都不应执行。
