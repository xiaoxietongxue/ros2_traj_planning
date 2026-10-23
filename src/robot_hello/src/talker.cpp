#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker"), count_(0)
  {
    // 发布频率可通过参数修改，方便练习：ros2 run robot_hello talker --ros-args -p publish_rate:=5.0
    publish_rate_ = this->declare_parameter<double>("publish_rate", 2.0);
    if (publish_rate_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_rate must be positive, fallback to 2.0 Hz");
      publish_rate_ = 2.0;
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("hello", 10);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Talker::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "Talker started, publishing /hello at %.2f Hz", publish_rate_);
  }

private:
  void on_timer()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from robot_hello: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_rate_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
