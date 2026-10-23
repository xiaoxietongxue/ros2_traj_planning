# ROS 2 基础概念

## ROS 2 是什么

ROS 2（Robot Operating System 2）不是传统意义上的操作系统，而是一套**机器人软件中间件**：提供进程间通信、包管理、启动工具、可视化接口和大量现成驱动。

可以把它理解成“机器人程序之间约定好的说话方式 + 一套开发工具”。

## 为什么需要它

机械臂项目里通常有很多独立程序同时运行：可视化、规划、控制器、传感器。它们需要交换数据，又不能全部写进一个巨大程序。ROS 2 让每个功能成为独立节点，用话题、服务、动作三种方式通信。

## ROS 2 和 ROS 1 的主要区别

| | ROS 1 | ROS 2 |
|---|---|---|
| 通信底层 | 自定义 + 中心化 Master | DDS，无强制 Master |
| 语言接口 | `roscpp` / `rospy` | `rclcpp` / `rclpy` |
| 构建 | `catkin` | `colcon` + `ament` |
| 启动文件 | `.launch`（XML） | `.launch.py`（Python） |
| 支持 | 已停止新发行版 | 当前主线 |

本虚拟机已有 ROS 1 风格的 `catkin_ws`，那是另一套系统。本项目只用 ROS 2 Humble，不要把两套命令混用。

## 节点、话题、发布者、订阅者

```mermaid
flowchart LR
    Talker["talker 节点<br/>Publisher"] -- "/hello<br/>std_msgs/String" --> Listener["listener 节点<br/>Subscriber"]
```

- **Node（节点）**：一个可运行的 ROS 2 程序。`talker` 和 `listener` 各是一个节点。
- **Topic（话题）**：带名字的数据通道。本示例是 `/hello`。
- **Publisher（发布者）**：往话题写数据。
- **Subscriber（订阅者）**：从话题读数据。
- **消息（Message）**：话题上流动的数据结构。本示例是 `std_msgs/String`，里面只有一个字符串字段 `data`。

一个话题可以有多个发布者和多个订阅者。发布者和订阅者不必同时启动；后启动的订阅者只能收到之后的消息。

本阶段只用 Topic。后面会遇到：

- **Service（服务）**：一问一答，例如“请计算一次 IK”。
- **Action（动作）**：长时间任务，带进度和取消，例如“请执行整条轨迹”。

## 工作空间和功能包

- **工作空间（workspace）**：一个总目录，例如 `~/robot_motion_ws`。里面有 `src`（源码）、`build`（中间文件）、`install`（安装结果）、`log`（编译日志）。
- **功能包（package）**：工作空间里的一个独立软件单元。必须包含 `package.xml` 和构建文件（本项目是 `CMakeLists.txt`）。

关系：一个工作空间可以有很多功能包；`colcon` 按依赖顺序编译它们。

## package.xml 和 CMakeLists.txt

| 文件 | 作用 |
|---|---|
| `package.xml` | 告诉 ROS：包名、依赖、许可证。`ros2 pkg`、`rosdep` 读它 |
| `CMakeLists.txt` | 告诉 CMake / 编译器：怎么把 `.cpp` 编成可执行文件 |

两者都要写依赖。只写一个，另一个漏了，常见后果是“能找到包但链接失败”，或“CMake 找不到依赖”。

## colcon

`colcon` 是 ROS 2 的构建工具。它遍历 `src` 下的功能包，依次调用 CMake 和编译器。

```bash
colcon build --symlink-install --packages-select robot_hello
```

- `colcon`：构建工具本身。
- `build`：执行编译。
- `--symlink-install`：安装时用符号链接，改 Python / launch 后不必每次完整重装。C++ 源文件改完仍需重新编译。
- `--packages-select robot_hello`：只编译这一个包，加快速度。

判断成功：终端最后出现 `Summary: 1 package finished`，并且没有红色 `Failed`。

## 环境变量

| 变量 | 含义 |
|---|---|
| `ROS_DISTRO` | 发行版名字，本机应为 `humble` |
| `AMENT_PREFIX_PATH` | ROS 2 搜索包的前缀列表 |
| `CMAKE_PREFIX_PATH` | CMake 搜索依赖的路径 |

`source /opt/ros/humble/setup.bash` 设置系统 ROS。`source install/setup.bash` 把**当前工作空间**叠加上去，之后才能 `ros2 run robot_hello talker`。
