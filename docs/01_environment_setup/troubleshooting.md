# Phase 1 常见问题

## `ros2: command not found`

当前终端没有加载 ROS 2 环境。

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
```

应输出 `humble`。若 `/opt/ros/humble` 不存在，说明发行版未安装，不要自行换成 Jazzy，先核对系统是否仍是 Ubuntu 22.04。

## `Package 'robot_hello' not found`

只 source 了系统 ROS，没有 source 工作空间。

```bash
source ~/robot_motion_ws/install/setup.bash
```

如果刚刚编译过，必须在**编译之后**重新 source。已经打开的旧终端不会自动更新。

## colcon 报 CMake 找不到 rclcpp

编译前没有 source ROS：

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_motion_ws
colcon build --symlink-install --packages-select robot_hello
```

## 编译权限错误 / 无法删除 build

曾经用 `sudo colcon build` 编译过。不要继续用 sudo。处理：

```bash
sudo chown -R parallels:parallels ~/robot_motion_ws
```

然后重新 `colcon build`。

## listener 没有任何输出

先确认 talker 在跑：

```bash
ros2 topic info /hello
ros2 topic echo /hello
```

如果 echo 也没有数据，是 talker 没起来。如果 echo 有、listener 没有，检查两个节点是不是订阅了同一个话题名（都应该是 `/hello`）。

ROS 2 的 DDS 在虚拟机里偶尔需要几秒发现对端，等 2 秒再看。

## `ros2 topic echo` 卡住没有消息

节点没启动，或当前终端和节点不在同一 ROS_DOMAIN_ID。本项目没有改过域 ID，保持默认 0。不要在一个终端里 `export ROS_DOMAIN_ID=1`。

## 改了 C++ 代码但运行结果没变

C++ 必须重新编译：

```bash
colcon build --symlink-install --packages-select robot_hello
```

已经在跑的节点不会自动重载，需要 Ctrl+C 后重新 `ros2 run`。

Launch 和 Python 文件在 `--symlink-install` 下一般不用重编，但仍要重启节点。

## 虚拟机里 RViz 打不开 / 很卡

Phase 1 不需要 RViz。若以后需要：优先检查 3D 加速，或降低 RViz 帧率。不要为此重装发行版。

## 想装 MoveIt 2 / Jazzy

当前系统是 Ubuntu 22.04 ARM64 + Humble。Jazzy 官方对应 Ubuntu 24.04。Phase 1 不要换发行版。MoveIt 2 放到 Phase 2，安装前先确认 aarch64 软件源里有对应包。
