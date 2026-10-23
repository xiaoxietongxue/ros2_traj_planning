#include "robot_motion_planning/motion_planner.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace robot_motion_planning
{

MotionPlanner::MotionPlanner(
  const rclcpp::Node::SharedPtr & node, const std::string & planning_group)
: node_(node),
  move_group_(node, planning_group)
{
  move_group_.setPlanningTime(10.0);
  move_group_.setNumPlanningAttempts(5);
  move_group_.setPlannerId("RRTConnectkConfigDefault");
  move_group_.setPoseReferenceFrame("base_link");
  move_group_.setEndEffectorLink("tool0");
  RCLCPP_INFO(
    node_->get_logger(), "MotionPlanner ready. group=%s ee=%s frame=%s",
    move_group_.getName().c_str(),
    move_group_.getEndEffectorLink().c_str(),
    move_group_.getPlanningFrame().c_str());
}

bool MotionPlanner::waitForCurrentState(double timeout_s)
{
  return static_cast<bool>(move_group_.getCurrentState(timeout_s));
}

std::string MotionPlanner::planningGroup() const
{
  return move_group_.getName();
}

std::vector<std::string> MotionPlanner::jointNames() const
{
  return move_group_.getJointNames();
}

std::vector<double> MotionPlanner::currentJointValues() const
{
  return move_group_.getCurrentJointValues();
}

geometry_msgs::msg::Pose MotionPlanner::currentPose() const
{
  return move_group_.getCurrentPose().pose;
}

PlanResult MotionPlanner::planToNamedTarget(const std::string & name)
{
  PlanResult result;
  if (!move_group_.setNamedTarget(name)) {
    result.message = "Unknown named target: " + name;
    return result;
  }
  const bool ok = static_cast<bool>(move_group_.plan(result.plan));
  return finalizePlan(result, ok, "named target " + name);
}

PlanResult MotionPlanner::planToJointTarget(const std::vector<double> & joints)
{
  PlanResult result;
  if (joints.size() != move_group_.getVariableCount()) {
    result.message = "Joint target size mismatch";
    return result;
  }
  if (!move_group_.setJointValueTarget(joints)) {
    result.message = "Invalid joint target";
    return result;
  }
  const bool ok = static_cast<bool>(move_group_.plan(result.plan));
  return finalizePlan(result, ok, "joint target");
}

PlanResult MotionPlanner::planToPoseTarget(const geometry_msgs::msg::Pose & pose)
{
  PlanResult result;
  move_group_.setPoseTarget(pose, "tool0");
  const bool ok = static_cast<bool>(move_group_.plan(result.plan));
  return finalizePlan(result, ok, "pose target");
}

PlanResult MotionPlanner::planCartesianPath(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, double eef_step)
{
  PlanResult result;
  if (waypoints.empty()) {
    result.message = "Cartesian waypoints are empty";
    return result;
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double fraction = move_group_.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  result.success = fraction > 0.99;
  result.waypoint_count = trajectory.joint_trajectory.points.size();
  result.plan.trajectory_ = trajectory;
  result.message = result.success
    ? "Cartesian path completed"
    : "Cartesian path only covered " + std::to_string(fraction * 100.0) + " percent";
  return result;
}

void MotionPlanner::addBoxObstacle(
  const std::string & id,
  const geometry_msgs::msg::Pose & pose,
  double size_x, double size_y, double size_z)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = "base_link";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {size_x, size_y, size_z};
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  planning_scene_.applyCollisionObject(object);
  RCLCPP_INFO(node_->get_logger(), "Added box obstacle '%s'", id.c_str());
}

void MotionPlanner::clearObstacles()
{
  const auto ids = planning_scene_.getKnownObjectNames();
  if (!ids.empty()) {
    planning_scene_.removeCollisionObjects(ids);
  }
}

PlanResult MotionPlanner::finalizePlan(
  PlanResult result, bool ok, const std::string & context)
{
  result.success = ok;
  result.planning_time_s = result.plan.planning_time_;
  result.waypoint_count = result.plan.trajectory_.joint_trajectory.points.size();
  result.message = ok ? "Planned " + context : "Failed to plan " + context;
  return result;
}

}  // namespace robot_motion_planning
