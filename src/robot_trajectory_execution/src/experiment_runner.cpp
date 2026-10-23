#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <rclcpp/rclcpp.hpp>

#include "robot_kinematics/arm_kinematics.hpp"
#include "robot_trajectory_execution/trajectory_checks.hpp"
#include "robot_trajectory_optimization/minco.hpp"
#include "robot_trajectory_optimization/minimum_jerk.hpp"
#include "robot_trajectory_optimization/quintic_trajectory.hpp"

namespace
{

struct Row
{
  std::string experiment;
  std::string algorithm;
  double compute_ms{0.0};
  robot_trajectory::TrajectoryMetrics metrics;
  robot_trajectory_execution::LimitReport limits;
  robot_trajectory_execution::CollisionReport collision;
};

template <typename Maker>
Row evaluate(
  const std::string & experiment,
  const std::string & algorithm,
  planning_scene::PlanningScene & scene,
  Maker maker)
{
  Row row;
  row.experiment = experiment;
  row.algorithm = algorithm;
  const auto start = std::chrono::steady_clock::now();
  auto trajectory = maker();
  row.compute_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  row.metrics = robot_trajectory::evaluateTrajectory(*trajectory, 0.02);
  row.limits = robot_trajectory_execution::checkLimits(
    scene.getRobotModel(), "ur_manipulator", *trajectory, 0.02);
  row.collision = robot_trajectory_execution::checkCollision(
    scene, "ur_manipulator", *trajectory, 0.02);
  return row;
}

void writeCsv(const std::string & path, const std::vector<Row> & rows)
{
  std::ofstream out(path);
  out << "experiment,algorithm,compute_ms,trajectory_time_s,jerk_cost,max_velocity,max_acceleration,"
         "max_jerk,continuity,position_ok,velocity_ok,acceleration_ok,collision_free,colliding_samples\n";
  out.setf(std::ios::fixed);
  out.precision(6);
  for (const auto & row : rows) {
    out << row.experiment << "," << row.algorithm << ","
        << row.compute_ms << ","
        << row.metrics.duration << ","
        << row.metrics.jerk_cost << ","
        << row.metrics.max_velocity << ","
        << row.metrics.max_acceleration << ","
        << row.metrics.max_jerk << ","
        << (row.metrics.continuity.ok ? "true" : "false") << ","
        << (row.limits.position_ok ? "true" : "false") << ","
        << (row.limits.velocity_ok ? "true" : "false") << ","
        << (row.limits.acceleration_ok ? "true" : "false") << ","
        << (row.collision.collision_free ? "true" : "false") << ","
        << row.collision.colliding_samples << "\n";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("experiment_runner");
  std::shared_ptr<robot_kinematics::ArmKinematics> kinematics;
  try {
    kinematics = robot_kinematics::ArmKinematics::createUr5e(node);
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  auto scene = std::make_shared<planning_scene::PlanningScene>(kinematics->robotModel());
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd start(6);
  Eigen::VectorXd mid(6);
  Eigen::VectorXd goal(6);
  Eigen::VectorXd extra(6);
  start << 0.3, -1.0, 1.2, -1.5, 1.2, 0.5;
  mid << 0.0, -1.2, 1.4, -1.3, 0.9, 0.2;
  goal << -0.4, -0.8, 0.9, -1.0, 1.1, -0.3;
  extra << -0.2, -1.1, 1.1, -1.4, 0.7, 0.1;

  const std::vector<Eigen::VectorXd> single{start, goal};
  const std::vector<Eigen::VectorXd> multi{start, mid, goal, extra};
  const std::vector<double> single_time{2.0};
  const std::vector<double> multi_time{1.2, 1.2, 1.2};

  std::vector<Row> rows;
  auto run_suite = [&](const std::string & name, const std::vector<Eigen::VectorXd> & waypoints,
      const std::vector<double> & times, double total_time) {
      rows.push_back(evaluate(name, "quintic", *scene, [&]() {
          auto trajectory = std::make_shared<robot_trajectory::JointQuinticTrajectory>();
          std::string error;
          if (!trajectory->generate(waypoints, times, zero, zero, zero, zero, error)) {
            throw std::runtime_error(error);
          }
          return trajectory;
        }));
      rows.push_back(evaluate(name, "minimum_jerk", *scene, [&]() {
          auto trajectory = std::make_shared<robot_trajectory::MinimumJerkTrajectory>();
          std::string error;
          if (!trajectory->generate(waypoints, times, zero, zero, zero, zero, error)) {
            throw std::runtime_error(error);
          }
          return trajectory;
        }));
      rows.push_back(evaluate(name, "minco", *scene, [&]() {
          auto trajectory = std::make_shared<robot_trajectory::MincoTrajectory>();
          std::string error;
          if (!trajectory->optimizeTimeAllocation(
              waypoints, total_time, zero, zero, zero, zero, {}, error))
          {
            throw std::runtime_error(error);
          }
          return trajectory;
        }));
    };

  run_suite("single_target", single, single_time, 2.0);
  run_suite("multi_waypoint", multi, multi_time, 3.6);

  robot_trajectory::JointQuinticTrajectory probe;
  std::string error;
  if (!probe.generate(multi, multi_time, zero, zero, zero, zero, error)) {
    std::cerr << error << "\n";
    kinematics.reset();
    rclcpp::shutdown();
    return 1;
  }
  robot_kinematics::EndEffectorPose pose;
  if (!kinematics->computeFK(probe.position(0.6), pose, error)) {
    std::cerr << error << "\n";
    kinematics.reset();
    rclcpp::shutdown();
    return 1;
  }
  robot_trajectory_execution::addBox(*scene, "blocking_box", pose.position, Eigen::Vector3d(0.05, 0.05, 0.05));
  std::cout << "obstacle_box_m " << pose.position.transpose() << "\n";
  run_suite("obstacle", multi, multi_time, 3.6);

  const std::string csv = "/home/parallels/robot_motion_ws/results/phase8_metrics.csv";
  writeCsv(csv, rows);
  std::cout << "acceleration_threshold_rad_s2 " << robot_trajectory_execution::kAccelerationThreshold << "\n";
  for (const auto & row : rows) {
    std::cout << row.experiment << " " << row.algorithm
              << " compute_ms=" << row.compute_ms
              << " jerk=" << row.metrics.jerk_cost
              << " max_v=" << row.metrics.max_velocity
              << " max_a=" << row.metrics.max_acceleration
              << " pos=" << row.limits.position_ok
              << " vel=" << row.limits.velocity_ok
              << " acc=" << row.limits.acceleration_ok
              << " collision_free=" << row.collision.collision_free
              << " hits=" << row.collision.colliding_samples
              << "\n";
    if (!row.limits.detail.empty()) {
      std::cout << "  " << row.limits.detail << "\n";
    }
  }
  std::cout << "wrote " << csv << "\n";
  kinematics.reset();
  scene.reset();
  rclcpp::shutdown();
  return 0;
}
