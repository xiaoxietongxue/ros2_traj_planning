# Phase 3 开发报告

日期：2026-10-23  
工作空间：`/home/parallels/robot_motion_ws`

## 已实现功能

- 功能包 `robot_kinematics`。
- 用官方 `ur_description` / `ur_moveit_config` 加载 UR5e，规划组 `ur_manipulator`，末端 `tool0`。
- `computeFK`、`computeIK`、`computeJacobian`、`computeManipulability`。
- 拒绝错误维度、NaN、超关节限位的输入。
- IK 失败和超限解都返回 `false`。
- 可操作度低于 `1e-3` 时标记靠近奇异。
- `kinematics_demo` 做 FK/IK 往返，并比较普通构型与 `wrist_2 = 0`。
- 6 个单元测试。

没有手写 DH，没有把五次多项式叫成 MINCO，没有进入轨迹生成。

## 主要文件

| 路径 | 作用 |
|---|---|
| `src/robot_kinematics/include/robot_kinematics/arm_kinematics.hpp` | 接口 |
| `src/robot_kinematics/src/arm_kinematics.cpp` | 实现 |
| `src/robot_kinematics/src/kinematics_demo.cpp` | 验收 |
| `src/robot_kinematics/test/test_kinematics.cpp` | 测试 |
| `docs/03_robot_kinematics/tutorial.md` | 教程 |
| `docs/03_robot_kinematics/mathematics.md` | 公式 |

## 核心技术

MoveIt `RobotModel` / `RobotState` 做 FK 和雅可比。IK 用 MoveIt 自带的 KDL 插件（BSD 许可证的 MoveIt 组件）。可操作度在本包内用 Eigen 计算。URDF 仍是 Universal Robots 官方描述。

## 编译结果

```text
Starting >>> robot_kinematics
Finished <<< robot_kinematics [9.22s]
Summary: 1 package finished [9.34s]
```

## 测试结果

单元测试（实际运行）：

```text
[  PASSED  ] 6 tests.
1/1 Test #1: test_kinematics  Passed  0.35 sec
100% tests passed, 0 tests failed out of 1
```

`kinematics_demo`（实际运行）：

```text
group: ur_manipulator  tip: tool0  model_frame: world  dof: 6
regular position_m: 0.6520 0.3790 0.5050
regular manipulability: 0.102818
singular position_m: 0.4919 0.2329 0.5875
singular manipulability: 0.000000
FK/IK position error_m: 0.000000
FK/IK orientation error_rad: 0.000000
Acceptance: FK, Jacobian, manipulability and FK/IK consistency passed
```

这是计算验证，没有打开 RViz。

## 已知问题

1. 可操作度混合了米和弧度，只能在这台 UR5e 上做相对比较。
2. KDL 是数值 IK。换种子可能得到另一组关节，位姿仍应一致。
3. 位置在 `world`，不是 `base_link`。
4. 关节限位来自带 `safety_pos_margin:=0.15` 的 URDF，比手册标称范围略窄。

## 下一阶段预告（未开始）

Phase 4：五次多项式轨迹，输出位置、速度、加速度和 jerk，并写成 CSV。  
**等待确认后再开始。**
