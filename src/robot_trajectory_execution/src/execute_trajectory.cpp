#include <chrono>
#include <iostream>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot_trajectory_optimization/quintic_trajectory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("execute_trajectory");
  using Action = control_msgs::action::FollowJointTrajectory;
  auto client = rclcpp_action::create_client<Action>(node, "/joint_trajectory_controller/follow_joint_trajectory");
  if (!client->wait_for_action_server(std::chrono::seconds(30))) {
    std::cerr << "FollowJointTrajectory action server not available\n";
    rclcpp::shutdown();
    return 1;
  }

  const std::vector<std::string> names{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  Eigen::VectorXd start(6);
  Eigen::VectorXd goal(6);
  start << 0.0, -1.57, 0.0, -1.57, 0.0, 0.0;
  goal << 0.4, -1.40, 0.5, -1.20, 0.6, 0.2;
  robot_trajectory::JointQuinticTrajectory trajectory;
  std::string error;
  if (!trajectory.generate(
      {start, goal}, {2.5}, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6),
      Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), error))
  {
    std::cerr << error << "\n";
    rclcpp::shutdown();
    return 1;
  }

  Action::Goal goal_msg;
  const auto samples = robot_trajectory::sampleTrajectory(trajectory, 0.02);
  goal_msg.trajectory.joint_names = names;
  for (const auto & sample : samples) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(sample.position.data(), sample.position.data() + sample.position.size());
    point.velocities.assign(sample.velocity.data(), sample.velocity.data() + sample.velocity.size());
    point.time_from_start = rclcpp::Duration::from_seconds(sample.time);
    goal_msg.trajectory.points.push_back(point);
  }

  rclcpp_action::ClientGoalHandle<Action>::SharedPtr handle;
  for (int attempt = 0; attempt < 20 && !handle; ++attempt) {
    auto future = client->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3)) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      handle = future.get();
    }
    if (!handle) {
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
  }
  if (!handle) {
    std::cerr << "failed to send trajectory goal\n";
    rclcpp::shutdown();
    return 1;
  }
  auto result_future = client->async_get_result(handle);
  if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(15)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cerr << "trajectory execution timed out\n";
    rclcpp::shutdown();
    return 1;
  }
  const auto result = result_future.get();
  std::cout << "execution_code=" << static_cast<int>(result.code) << "\n";
  sensor_msgs::msg::JointState::SharedPtr latest;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&latest](const sensor_msgs::msg::JointState::SharedPtr msg) { latest = msg; });
  rclcpp::spin_some(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!latest && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
  }
  if (latest) {
    std::cout << "final_joint_states";
    for (double value : latest->position) {
      std::cout << " " << value;
    }
    std::cout << "\n";
  } else {
    std::cout << "final_joint_states unavailable\n";
  }
  const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
