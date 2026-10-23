#ifndef ROBOT_MOTION_PLANNING_MOTION_PLANNER_HPP_
#define ROBOT_MOTION_PLANNING_MOTION_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_motion_planning
{

struct PlanResult
{
  bool success{false};
  std::string message;
  double planning_time_s{0.0};
  std::size_t waypoint_count{0};
  moveit::planning_interface::MoveGroupInterface::Plan plan;
};

class MotionPlanner
{
public:
  MotionPlanner(const rclcpp::Node::SharedPtr & node, const std::string & planning_group);

  bool waitForCurrentState(double timeout_s);
  std::string planningGroup() const;
  std::vector<std::string> jointNames() const;
  std::vector<double> currentJointValues() const;
  geometry_msgs::msg::Pose currentPose() const;

  PlanResult planToNamedTarget(const std::string & name);
  PlanResult planToJointTarget(const std::vector<double> & joints);
  PlanResult planToPoseTarget(const geometry_msgs::msg::Pose & pose);
  PlanResult planCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double eef_step = 0.01);

  void addBoxObstacle(
    const std::string & id,
    const geometry_msgs::msg::Pose & pose,
    double size_x, double size_y, double size_z);
  void clearObstacles();

private:
  PlanResult finalizePlan(PlanResult result, bool ok, const std::string & context);

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
};

}  // namespace robot_motion_planning

#endif  // ROBOT_MOTION_PLANNING_MOTION_PLANNER_HPP_
