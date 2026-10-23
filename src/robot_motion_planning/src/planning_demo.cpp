#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "robot_motion_planning/motion_planner.hpp"

using robot_motion_planning::MotionPlanner;
using robot_motion_planning::PlanResult;

namespace
{

void log_result(const rclcpp::Logger & logger, const std::string & title, const PlanResult & result)
{
  RCLCPP_INFO(
    logger, "[%s] success=%s time=%.3fs waypoints=%zu msg=%s",
    title.c_str(), result.success ? "true" : "false",
    result.planning_time_s, result.waypoint_count, result.message.c_str());
}

geometry_msgs::msg::Pose offset_pose(geometry_msgs::msg::Pose pose, double dx, double dy, double dz)
{
  pose.position.x += dx;
  pose.position.y += dy;
  pose.position.z += dz;
  return pose;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("planning_demo", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // move_group 需要几秒完成参数与场景初始化
  rclcpp::sleep_for(std::chrono::seconds(4));

  int failures = 0;
  try {
    MotionPlanner planner(node, "ur_manipulator");
    const auto logger = node->get_logger();

    if (!planner.waitForCurrentState(10.0)) {
      throw std::runtime_error("Timed out waiting for /joint_states");
    }

    const auto joints = planner.currentJointValues();
    const auto pose0 = planner.currentPose();
    if (joints.size() < 6) {
      throw std::runtime_error("Current joint vector is shorter than 6");
    }
    RCLCPP_INFO(logger, "Current joints: %.3f %.3f %.3f %.3f %.3f %.3f",
      joints.size() > 5 ? joints[0] : 0.0,
      joints.size() > 5 ? joints[1] : 0.0,
      joints.size() > 5 ? joints[2] : 0.0,
      joints.size() > 5 ? joints[3] : 0.0,
      joints.size() > 5 ? joints[4] : 0.0,
      joints.size() > 5 ? joints[5] : 0.0);
    RCLCPP_INFO(logger, "Current EE pose: [%.3f, %.3f, %.3f]",
      pose0.position.x, pose0.position.y, pose0.position.z);

    auto named = planner.planToNamedTarget("up");
    log_result(logger, "named:up", named);
    failures += named.success ? 0 : 1;

    std::vector<double> side = joints;
    if (side.size() >= 1) {
      side[0] = 0.4;
    }
    auto joint = planner.planToJointTarget(side);
    log_result(logger, "joint:side", joint);
    failures += joint.success ? 0 : 1;

    auto pose_target = offset_pose(pose0, 0.0, 0.10, -0.08);
    auto pose = planner.planToPoseTarget(pose_target);
    log_result(logger, "pose:offset", pose);
    failures += pose.success ? 0 : 1;

    std::vector<geometry_msgs::msg::Pose> cartesian_wps = {
      offset_pose(pose0, 0.0, 0.0, -0.05),
      offset_pose(pose0, 0.05, 0.0, -0.05),
    };
    auto cartesian = planner.planCartesianPath(cartesian_wps, 0.01);
    log_result(logger, "cartesian", cartesian);
    failures += cartesian.success ? 0 : 1;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.50;
    box_pose.position.y = -0.20;
    box_pose.position.z = 0.20;
    planner.addBoxObstacle("demo_box", box_pose, 0.12, 0.12, 0.40);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    auto around = planner.planToPoseTarget(pose_target);
    log_result(logger, "pose:with_obstacle", around);
    failures += around.success ? 0 : 1;

    geometry_msgs::msg::Pose into_box = box_pose;
    into_box.orientation = pose0.orientation;
    auto colliding = planner.planToPoseTarget(into_box);
    log_result(logger, "pose:into_box", colliding);
    if (colliding.success) {
      RCLCPP_ERROR(logger, "Planning into the box should have failed");
      failures += 1;
    } else {
      RCLCPP_INFO(logger, "Collision checking rejected the in-box target (expected)");
    }

    planner.clearObstacles();
    RCLCPP_INFO(logger, "Demo finished with %d failed case(s)", failures);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "planning_demo exception: %s", ex.what());
    failures = 99;
  }

  rclcpp::shutdown();
  spinner.join();
  return failures == 0 ? 0 : 1;
}
