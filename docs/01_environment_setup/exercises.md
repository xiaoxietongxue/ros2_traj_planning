# Phase 1 练习

先独立完成，再看 [solutions.md](solutions.md)。

## 练习 1：修改话题发布频率

`talker` 有参数 `publish_rate`，默认 2.0 Hz。

不改源码，把它改成 5 Hz 运行，并用 `ros2 topic hz /hello` 确认频率大约是 5。

提示：`ros2 run` 可以用 `--ros-args -p 参数名:=值`。

## 练习 2：修改消息内容

把发布字符串改成包含你的名字或任意前缀，例如 `UR5e demo: 3`。

需要改 `talker.cpp`，重新编译，再运行验证。

## 练习 3：使用 ros2 topic echo 查看消息

1. 只启动 talker。
2. 用 `ros2 topic echo /hello` 打印消息。
3. 用 `ros2 topic info /hello -v` 查看发布者数量。
4. 再启动 listener，再次查看订阅者数量是否增加。

把你看到的 Publisher / Subscriber 数量记下来。
