# ROS 2 + MoveIt 2 + MINCO 机械臂运动规划与轨迹优化

面向初学者的机器人运动规划教学与工程实践平台。开发方式为 **Tutorial-Driven Development**：每个阶段都能编译、运行、验证，并同步产出中文教程。

当前进度：**Phase 2 已完成**（UR5e + MoveIt 2 规划，无界面 demo 通过）。Phase 3 需确认后再开始。

## 当前环境（实测）

| 项目 | 实际值 |
|---|---|
| 系统 | Ubuntu 22.04.2 LTS（Parallels 虚拟机） |
| CPU 架构 | aarch64（Apple Silicon） |
| ROS 2 | Humble（已有环境，未覆盖） |
| C++ | g++ 11.4，C++17 |
| Eigen | 3.4.0 |
| RViz 2 | 已安装 |
| MoveIt 2 | 2.5.10（Phase 2 已从 apt 安装） |
| ros2_control | 已随 MoveIt 依赖安装；本阶段不启动控制器 |
| OMPL | 已随 `ros-humble-moveit-planners-ompl` 安装 |

计划中的 Ubuntu 24.04 + Jazzy 是推荐组合，**不是当前机器配置**。本仓库按 Humble 开发，避免破坏已有环境。

## 快速开始（Phase 1）

```bash
cd ~/robot_motion_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_hello
source install/setup.bash
```

开两个终端：

```bash
# 终端 1
source ~/robot_motion_ws/install/setup.bash
ros2 run robot_hello talker

# 终端 2
source ~/robot_motion_ws/install/setup.bash
ros2 run robot_hello listener
```

或一条命令同时启动：

```bash
source ~/robot_motion_ws/install/setup.bash
ros2 launch robot_hello hello.launch.py
```

第三个终端检查话题：

```bash
ros2 topic echo /hello
```

## 工作空间

当前只创建 Phase 1 真正用到的内容，不为后续阶段预建空包。

```
~/robot_motion_ws/
├── src/robot_hello/              Phase 1：C++ Publisher / Subscriber
├── src/robot_bringup/            Phase 2：启动 UR5e + MoveIt
├── src/robot_motion_planning/    Phase 2：规划 API 与 demo
├── docs/
├── README.md
├── PLAN.md
└── AGENTS.md
```

只看模型：

```bash
ros2 launch robot_bringup view_ur5e.launch.py
```

无界面规划验收：

```bash
ros2 launch robot_bringup ur5e_moveit.launch.py launch_rviz:=false run_demo:=true
```

## 教程入口

- [教程总目录](docs/README.md)
- [Phase 1：环境与 ROS 2 基础](docs/01_environment_setup/tutorial.md)
- [Phase 1 开发报告](docs/01_environment_setup/development_report.md)

## 阶段规划

详见 [PLAN.md](PLAN.md)。顺序固定为 Phase 1 → 8，完成一个阶段并经确认后再进入下一阶段。
