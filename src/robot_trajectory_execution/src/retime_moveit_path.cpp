#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "robot_motion_planning/motion_planner.hpp"
#include "robot_trajectory_execution/trajectory_checks.hpp"
#include "robot_trajectory_optimization/minco.hpp"
#include "robot_trajectory_optimization/minimum_jerk.hpp"
#include "robot_trajectory_optimization/quintic_trajectory.hpp"

namespace
{

std::vector<Eigen::VectorXd> waypointsFromPlan(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  std::vector<Eigen::VectorXd> waypoints;
  for (const auto & point : plan.trajectory_.joint_trajectory.points) {
    Eigen::VectorXd value(static_cast<Eigen::Index>(point.positions.size()));
    for (std::size_t i = 0; i < point.positions.size(); ++i) {
      value[static_cast<Eigen::Index>(i)] = point.positions[i];
    }
    if (waypoints.empty() || (waypoints.back() - value).norm() > 1e-4) {
      waypoints.push_back(value);
    }
  }
  return waypoints;
}

template <typename Trajectory>
void report(const std::string & name, const Trajectory & trajectory)
{
  const auto metrics = robot_trajectory::evaluateTrajectory(trajectory, 0.02);
  std::cout << name
            << " duration_s=" << metrics.duration
            << " jerk=" << metrics.jerk_cost
            << " max_v=" << metrics.max_velocity
            << " max_a=" << metrics.max_acceleration
            << " continuous=" << (metrics.continuity.ok ? "true" : "false")
            << "\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("retime_moveit_path", options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int exit_code = 0;
  try {
    rclcpp::sleep_for(std::chrono::seconds(4));
    robot_motion_planning::MotionPlanner planner(node, "ur_manipulator");
    if (!planner.waitForCurrentState(15.0)) {
      throw std::runtime_error("timed out waiting for joint states");
    }
    auto current = planner.currentJointValues();
    if (current.size() != 6) {
      throw std::runtime_error("expected 6 joints");
    }
    current[0] += 0.6;
    auto plan = planner.planToJointTarget(current);
    std::cout << "moveit_success=" << (plan.success ? "true" : "false")
              << " waypoints=" << plan.waypoint_count
              << " planning_time_s=" << plan.planning_time_s
              << " msg=" << plan.message << "\n";
    if (!plan.success) {
      throw std::runtime_error("MoveIt planning failed");
    }
    auto dense = waypointsFromPlan(plan.plan);
    auto waypoints = robot_trajectory_execution::downsampleWaypoints(dense, 6);
    std::cout << "dense_waypoints=" << dense.size()
              << " used_waypoints=" << waypoints.size() << "\n";
    const double total_time = std::max(2.0, 0.6 * static_cast<double>(waypoints.size() - 1));
    std::vector<double> times(waypoints.size() - 1, total_time / static_cast<double>(waypoints.size() - 1));
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(waypoints.front().size()));
    std::string error;
    robot_trajectory::JointQuinticTrajectory quintic;
    robot_trajectory::MinimumJerkTrajectory jerk;
    robot_trajectory::MincoTrajectory minco;
    if (!quintic.generate(waypoints, times, zero, zero, zero, zero, error) ||
      !jerk.generate(waypoints, times, zero, zero, zero, zero, error) ||
      !minco.optimizeTimeAllocation(waypoints, total_time, zero, zero, zero, zero, {}, error))
    {
      throw std::runtime_error(error);
    }
    report("quintic", quintic);
    report("minimum_jerk", jerk);
    report("minco", minco);
    const auto message = robot_trajectory_execution::toJointTrajectory(jerk, planner.jointNames(), 0.02);
    std::cout << "joint_trajectory_points=" << message.points.size()
              << " joints=" << message.joint_names.size() << "\n";
    std::cout << "note: geometric path was collision-free; retimed curves are checked in experiment_runner\n";
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << "\n";
    exit_code = 1;
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}
