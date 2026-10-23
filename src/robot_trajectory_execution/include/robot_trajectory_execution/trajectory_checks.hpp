#pragma once

#include <string>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "robot_trajectory_optimization/quintic_trajectory.hpp"

namespace robot_trajectory_execution
{

struct LimitReport
{
  bool position_ok{true};
  bool velocity_ok{true};
  bool acceleration_ok{true};
  double max_velocity{0.0};
  double max_acceleration{0.0};
  std::string detail;
};

struct CollisionReport
{
  bool collision_free{true};
  int colliding_samples{0};
  int sample_count{0};
};

// 加速度上限不是 UR 手册公开值。这里用工程阈值做事后检查，并在结果里标明。
constexpr double kAccelerationThreshold = 10.0;

LimitReport checkLimits(
  const moveit::core::RobotModelConstPtr & model,
  const std::string & group_name,
  const robot_trajectory::JointTrajectory & trajectory,
  double sample_dt,
  double acceleration_threshold = kAccelerationThreshold);

CollisionReport checkCollision(
  planning_scene::PlanningScene & scene,
  const std::string & group_name,
  const robot_trajectory::JointTrajectory & trajectory,
  double sample_dt);

void addBox(
  planning_scene::PlanningScene & scene,
  const std::string & id,
  const Eigen::Vector3d & center,
  const Eigen::Vector3d & size);

trajectory_msgs::msg::JointTrajectory toJointTrajectory(
  const robot_trajectory::JointTrajectory & trajectory,
  const std::vector<std::string> & joint_names,
  double sample_dt);

std::vector<Eigen::VectorXd> downsampleWaypoints(
  const std::vector<Eigen::VectorXd> & waypoints,
  int max_waypoints);

}  // namespace robot_trajectory_execution
