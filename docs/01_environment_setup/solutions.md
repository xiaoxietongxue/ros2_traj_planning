# Phase 1 练习参考答案

先自己做。以下是一种正确做法，不是唯一做法。

## 练习 1

```bash
source /opt/ros/humble/setup.bash
source ~/robot_motion_ws/install/setup.bash
ros2 run robot_hello talker --ros-args -p publish_rate:=5.0
```

另一个终端：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /hello
```

预期平均频率接近 5 Hz。虚拟机负载高时可能略低。

非法值（例如 `0` 或负数）时，talker 会警告并回退到 2.0 Hz。

## 练习 2

修改 `src/robot_hello/src/talker.cpp` 中这一行：

```cpp
message.data = "UR5e demo: " + std::to_string(count_++);
```

然后：

```bash
cd ~/robot_motion_ws
colcon build --symlink-install --packages-select robot_hello
source install/setup.bash
ros2 run robot_hello talker
```

`ros2 topic echo /hello` 应出现新前缀。做完练习可以把字符串改回 `Hello from robot_hello: `，以免和教程截图不一致。

## 练习 3

```bash
ros2 run robot_hello talker          # 终端 1
ros2 topic echo /hello               # 终端 2，应持续刷消息
ros2 topic info /hello -v            # 终端 3
```

只开 talker 时：Publisher count = 1，Subscription count = 0（echo 本身也是订阅者，开着 echo 时订阅者至少为 1）。

再开 listener 后，订阅者数量增加 1。
