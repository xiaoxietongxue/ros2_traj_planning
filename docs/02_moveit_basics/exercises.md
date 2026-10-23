# Phase 2 练习

先自己做，再看 [solutions.md](solutions.md)。

## 练习 1：改目标关节角

改 `planning_demo.cpp`，把 `side[0] = 0.4` 换成另一个在限位内的值（例如 `-0.6`），重新编译运行，确认 `joint:side` 仍成功。

## 练习 2：改规划器

`MotionPlanner` 构造函数里有：

```cpp
move_group_.setPlannerId("RRTConnectkConfigDefault");
```

改成 `RRTstarkConfigDefault` 或 `PRMkConfigDefault`（名字必须写在 `ompl_planning.yaml` 里）。比较三次规划时间。

## 练习 3：加障碍物并看路径变化

把 `demo_box` 的 `y` 从 `0.0` 改到 `0.15`，或把盒子加厚。观察 `pose:with_obstacle` 是成功、变慢，还是失败。在 RViz 里打开 launch_rviz，看场景中的盒子和规划曲线。
