# Linux 基础（本项目够用的部分）

完成本节后，你应当能在终端里进入目录、查看文件、复制路径，并理解 `sudo` 和权限。

## 终端是什么

终端是用文字和系统对话的窗口。ROS 2 的编译、启动节点、查看话题，几乎都在终端里完成。

打开方式：Ubuntu 桌面按 `Ctrl+Alt+T`。

## 路径

| 写法 | 含义 |
|---|---|
| `/` | 系统根目录 |
| `~` | 当前用户主目录，本虚拟机是 `/home/parallels` |
| `.` | 当前目录 |
| `..` | 上一级目录 |
| 绝对路径 | 从 `/` 写起，例如 `/opt/ros/humble` |
| 相对路径 | 从当前目录写起，例如 `src/robot_hello` |

本项目工作空间的绝对路径是 `/home/parallels/robot_motion_ws`，也可以写成 `~/robot_motion_ws`。

## 常用命令

| 命令 | 作用 | 例子 |
|---|---|---|
| `pwd` | 打印当前目录 | `pwd` |
| `ls` | 列出文件 | `ls -la` |
| `cd` | 进入目录 | `cd ~/robot_motion_ws` |
| `mkdir` | 创建目录 | `mkdir -p src` |
| `cat` | 打印文件内容 | `cat README.md` |
| `nano` / `code` | 编辑文件 | `nano src/robot_hello/src/talker.cpp` |
| `echo` | 打印一句话 | `echo $ROS_DISTRO` |
| `which` | 查找程序位置 | `which ros2` |

`mkdir -p` 里的 `-p` 表示：父目录不存在就一起创建，目录已存在也不报错。

## 权限与 sudo

有些操作需要管理员权限，例如用 `apt` 安装软件。在命令前加 `sudo`：

```bash
sudo apt install git
```

系统会问你的登录密码。密码输入时屏幕不会显示星号，直接输入后回车即可。

**不要**对 ROS 工作空间使用 `sudo colcon build`。用管理员身份编译，生成的文件会变成 root 所有，之后普通用户无法覆盖。

## 环境变量与 source

环境变量是“当前终端记住的配置”。ROS 2 的命令（`ros2`、`rviz2`）只有在加载了对应环境后才找得到。

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO    # 应输出 humble
```

`source` 的作用：在**当前终端**执行脚本，并把脚本里设置的变量留下来。新开一个终端后，这些变量不会自动存在，需要再 source 一次，或写进 `~/.bashrc`。

本虚拟机的 `~/.bashrc` 已经有：

```bash
source /opt/ros/humble/setup.bash
```

所以新开终端一般已经能用 `ros2`。编译自己的工作空间后，还要再 source 一次工作空间：

```bash
source ~/robot_motion_ws/install/setup.bash
```

## 练习

1. 用 `pwd` 和 `ls` 确认 `~/robot_motion_ws/src/robot_hello` 存在。
2. 用 `echo $ROS_DISTRO` 确认发行版是 `humble`。
3. 用 `which ros2` 查看 `ros2` 命令的位置。
