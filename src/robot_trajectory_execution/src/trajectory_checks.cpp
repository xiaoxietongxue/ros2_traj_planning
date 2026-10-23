#include "robot_trajectory_execution/trajectory_checks.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/duration.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace robot_trajectory_execution
{

LimitReport checkLimits(
  const moveit::core::RobotModelConstPtr & model,
  const std::string & group_name,
  const robot_trajectory::JointTrajectory & trajectory,
  double sample_dt,
  double acceleration_threshold)
{
  LimitReport report;
  const auto * group = model->getJointModelGroup(group_name);
  const auto names = group->getActiveJointModelNames();
  std::vector<double> lower(names.size());
  std::vector<double> upper(names.size());
  std::vector<double> velocity_limit(names.size(), 1e9);
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto & bounds = group->getJointModel(names[i])->getVariableBounds();
    lower[i] = bounds[0].min_position_;
    upper[i] = bounds[0].max_position_;
    if (bounds[0].velocity_bounded_) {
      velocity_limit[i] = std::min(std::abs(bounds[0].max_velocity_), std::abs(bounds[0].min_velocity_));
    }
  }

  const auto samples = robot_trajectory::sampleTrajectory(trajectory, sample_dt);
  std::ostringstream detail;
  for (const auto & sample : samples) {
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (sample.position[static_cast<Eigen::Index>(i)] < lower[i] - 1e-6 ||
        sample.position[static_cast<Eigen::Index>(i)] > upper[i] + 1e-6)
      {
        report.position_ok = false;
        detail << names[i] << " position exceeded at t=" << sample.time << "; ";
      }
      const double speed = std::abs(sample.velocity[static_cast<Eigen::Index>(i)]);
      report.max_velocity = std::max(report.max_velocity, speed);
      if (speed > velocity_limit[i] + 1e-6) {
        report.velocity_ok = false;
        detail << names[i] << " velocity exceeded at t=" << sample.time << "; ";
      }
      const double acceleration = std::abs(sample.acceleration[static_cast<Eigen::Index>(i)]);
      report.max_acceleration = std::max(report.max_acceleration, acceleration);
      if (acceleration > acceleration_threshold) {
        report.acceleration_ok = false;
        detail << names[i] << " acceleration exceeded at t=" << sample.time << "; ";
      }
    }
  }
  report.detail = detail.str();
  return report;
}

CollisionReport checkCollision(
  planning_scene::PlanningScene & scene,
  const std::string & group_name,
  const robot_trajectory::JointTrajectory & trajectory,
  double sample_dt)
{
  CollisionReport report;
  const auto * group = scene.getRobotModel()->getJointModelGroup(group_name);
  moveit::core::RobotState state(scene.getRobotModel());
  state.setToDefaultValues();
  const auto samples = robot_trajectory::sampleTrajectory(trajectory, sample_dt);
  report.sample_count = static_cast<int>(samples.size());
  collision_detection::CollisionRequest request;
  for (const auto & sample : samples) {
    std::vector<double> values(sample.position.data(), sample.position.data() + sample.position.size());
    state.setJointGroupPositions(group, values);
    state.update();
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, state);
    if (result.collision) {
      ++report.colliding_samples;
    }
  }
  report.collision_free = report.colliding_samples == 0;
  return report;
}

void addBox(
  planning_scene::PlanningScene & scene,
  const std::string & id,
  const Eigen::Vector3d & center,
  const Eigen::Vector3d & size)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = scene.getRobotModel()->getModelFrame();
  object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size.x(), size.y(), size.z()};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = center.x();
  pose.position.y = center.y();
  pose.position.z = center.z();
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  scene.processCollisionObjectMsg(object);
}

trajectory_msgs::msg::JointTrajectory toJointTrajectory(
  const robot_trajectory::JointTrajectory & trajectory,
  const std::vector<std::string> & joint_names,
  double sample_dt)
{
  trajectory_msgs::msg::JointTrajectory message;
  message.joint_names = joint_names;
  for (const auto & sample : robot_trajectory::sampleTrajectory(trajectory, sample_dt)) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(sample.position.data(), sample.position.data() + sample.position.size());
    point.velocities.assign(sample.velocity.data(), sample.velocity.data() + sample.velocity.size());
    point.accelerations.assign(
      sample.acceleration.data(), sample.acceleration.data() + sample.acceleration.size());
    point.time_from_start = rclcpp::Duration::from_seconds(sample.time);
    message.points.push_back(std::move(point));
  }
  return message;
}

std::vector<Eigen::VectorXd> downsampleWaypoints(
  const std::vector<Eigen::VectorXd> & waypoints,
  int max_waypoints)
{
  if (max_waypoints < 2 || static_cast<int>(waypoints.size()) <= max_waypoints) {
    return waypoints;
  }
  std::vector<Eigen::VectorXd> sampled;
  sampled.reserve(static_cast<std::size_t>(max_waypoints));
  for (int i = 0; i < max_waypoints; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(max_waypoints - 1);
    const std::size_t index = static_cast<std::size_t>(std::llround(alpha * (waypoints.size() - 1)));
    if (sampled.empty() || index != 0) {
      sampled.push_back(waypoints[std::min(index, waypoints.size() - 1)]);
    }
  }
  if ((sampled.back() - waypoints.back()).norm() > 1e-12) {
    sampled.back() = waypoints.back();
  }
  return sampled;
}

}  // namespace robot_trajectory_execution
