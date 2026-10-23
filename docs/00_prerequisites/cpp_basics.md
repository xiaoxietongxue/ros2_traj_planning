# C++ 基础（读本项目代码够用的部分）

第一次读 `talker.cpp` 时，下面这些概念最容易卡住。

## 头文件与源文件

- `.hpp` / `.h`：声明（有哪些函数、类）。
- `.cpp`：实现（函数具体怎么做）。

`#include "rclcpp/rclcpp.hpp"` 的意思：把 ROS 2 C++ 客户端库的声明引进来。没有它，编译器不认识 `rclcpp::Node`。

系统头文件用尖括号，例如 `#include <string>`；项目或库头文件常用引号。

## 命名空间

`std::string` 里的 `std` 是标准库命名空间，`::` 表示“里面的”。`rclcpp::Node` 同理。

`using namespace std::chrono_literals;` 只是为了能写 `1s`、`500ms` 这种时间字面量。本项目没有写 `using namespace std;`，避免名字冲突。

## 类与对象

```cpp
class Talker : public rclcpp::Node
```

- `class Talker`：自定义一种节点类型。
- `: public rclcpp::Node`：Talker **继承** ROS 2 节点，因此可以使用日志、参数、创建话题。
- `Talker()` 是构造函数，创建对象时自动执行。
- `main` 里的 `std::make_shared<Talker>()` 会在堆上创建一个 Talker 对象。

## 指针、引用与智能指针

| 写法 | 含义 |
|---|---|
| `int *p` | 普通指针，需要自己管理内存 |
| `int &r` | 引用，是已有变量的别名 |
| `std::shared_ptr<T>` | 智能指针，多个持有者共享同一对象，最后一个释放时自动销毁 |

ROS 2 里几乎到处都是智能指针：

```cpp
std::shared_ptr<rclcpp::Node>
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
```

原因：节点、发布者、订阅者的生命周期由 ROS 2 执行器管理，用 `shared_ptr` 可以避免“节点还在用，对象已经被删掉”。

`std::make_shared<Talker>()` 比 `new Talker()` 更安全，也是推荐写法。

## const

`const` 表示“这个东西在这里不会被修改”。

```cpp
void on_message(const std_msgs::msg::String::SharedPtr msg) const
```

- 参数前的 `const`：回调里不改消息内容。
- 函数后的 `const`：这个成员函数不改 Listener 自己的成员变量。

## STL 容器

标准库容器本阶段几乎用不到。后面轨迹模块会大量使用：

- `std::vector<double>`：可变长度数组，存关节角、采样点。
- `std::string`：字符串。
- `std::size_t`：表示数量的无符号整数。

## Eigen（本阶段未使用）

Eigen 是线性代数库。后续运动学和轨迹优化用它存向量和矩阵，例如 `Eigen::VectorXd`。头文件一般在 `/usr/include/eigen3`。

## CMake 在这里做什么

C++ 不能像 Python 那样直接运行源文件。CMake 根据 `CMakeLists.txt` 告诉编译器：

1. 这个工程叫什么。
2. 依赖哪些库（`rclcpp`、`std_msgs`）。
3. 要生成哪些可执行文件（`talker`、`listener`）。
4. 编译产物安装到哪里。

`colcon` 会调用 CMake，再调用 `g++`。你通常只需要记住：

```bash
colcon build --symlink-install --packages-select robot_hello
```
