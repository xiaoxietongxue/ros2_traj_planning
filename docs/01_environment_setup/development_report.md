# Phase 1 开发报告

日期：2026-10-23  
工作空间：`/home/parallels/robot_motion_ws`  
虚拟机：Parallels Desktop / Ubuntu 22.04.2 ARM64

## 环境检查结果（实测）

| 项目 | 结果 |
|---|---|
| `uname -m` | `aarch64` |
| `lsb_release` | Ubuntu 22.04.2 LTS (jammy) |
| `ROS_DISTRO` | `humble` |
| `which ros2` | `/opt/ros/humble/bin/ros2` |
| `which colcon` | `/bin/colcon` |
| `which rviz2` | `/opt/ros/humble/bin/rviz2` |
| Eigen | `libeigen3-dev` 3.4.0 |
| g++ | 11.4.0 |
| cmake | 3.22.1 |
| Python | 3.10.12 |
| MoveIt 2 | **未安装**（0 个 `ros-humble-moveit*` 包） |
| ros2_control | **未安装**（仅有无关的 `pendulum-control`） |
| OMPL | **未安装** |
| 已有工作空间 | `/home/parallels/ros2_ws`（含 `lidar_tutorial`）、`catkin_ws`，均未改动 |

决策：复用 Humble，不安装 Jazzy，不覆盖已有环境。Phase 1 只额外安装了 `git`（原先缺失）。MoveIt / OMPL / ros2_control 留到对应阶段再评估 ARM64 软件源。

## 本阶段学习内容

- ROS 2 节点、话题、发布者、订阅者
- 工作空间与功能包
- `package.xml`、`CMakeLists.txt`、`colcon`
- 环境变量与 `source`

教程路径：

- `docs/README.md`
- `docs/00_prerequisites/`
- `docs/01_environment_setup/tutorial.md`

## 实际完成的工程功能

- 创建 `~/robot_motion_ws` 并初始化 Git
- C++ 功能包 `robot_hello`
  - `talker`：向 `/hello` 发布 `std_msgs/String`，默认 2 Hz，参数 `publish_rate`
  - `listener`：订阅并打印
  - `hello.launch.py`：同时启动两者
- 未预建后续阶段的空包

## 编译结果

```text
Starting >>> robot_hello
Finished <<< robot_hello [4.03s]
Summary: 1 package finished [4.14s]
```

`ros2 pkg prefix robot_hello` → `/home/parallels/robot_motion_ws/install/robot_hello`

首次编译曾因 `package.xml` 邮箱 `parallels@local` 无法通过 catkin 校验而失败，已改为 `parallels@example.com`。

## 测试结果（实际运行）

同时启动 talker 与 listener 后：

```text
ros2 topic list          → /hello
ros2 topic info /hello   → Type: std_msgs/msg/String
                           Publisher count: 1
                           Subscription count: 1
ros2 topic echo --once   → data: 'Hello from robot_hello: 7'
```

日志对应关系（摘录）：

```text
[talker]   Publishing: 'Hello from robot_hello: 7'
[listener] I heard: 'Hello from robot_hello: 7'
```

时间戳间隔约 0.5 s，与默认 2 Hz 一致。`ros2 topic hz` 因节点在采样结束前退出，未拿到稳定统计，频率以日志间隔为准。

## 练习任务

见 `docs/01_environment_setup/exercises.md`：

1. 用参数把频率改成 5 Hz
2. 修改消息字符串并重新编译
3. 用 `ros2 topic echo` / `topic info` 观察发布者和订阅者数量

## 已知问题

1. 虚拟机为 ARM64 + Humble，与计划中的 Ubuntu 24.04 + Jazzy 不一致。后续装 MoveIt 2 必须先查 aarch64 软件包。
2. 本机原先没有 git，已安装 2.34.1；未改系统 Python，未动已有工作空间。
3. 未在图形界面里点选运行 RViz（Phase 1 不需要）。`rviz2` 可执行文件存在。
4. `prlctl exec` 默认以 root 运行。日常开发请用用户 `parallels` 打开终端，不要 `sudo colcon build`。

## 运行命令

```bash
cd ~/robot_motion_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_hello talker
# 另一个终端
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_hello listener
```

或：`ros2 launch robot_hello hello.launch.py`

## 下一阶段预告（未开始）

Phase 2：集成 UR5e、配置 MoveIt 2、在 RViz 2 中做关节目标与末端位姿规划。  
**等待确认后再开始。**
