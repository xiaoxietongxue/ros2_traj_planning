#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_trajectory_optimization/minco.hpp"
#include "robot_trajectory_optimization/minimum_jerk.hpp"
#include "robot_trajectory_optimization/quintic_trajectory.hpp"

namespace
{

struct Built
{
  std::string name;
  double compute_ms{0.0};
  robot_trajectory::TrajectoryMetrics metrics;
  bool csv_ok{false};
};

template <typename Builder>
Built run(
  const std::string & name,
  const std::string & csv_path,
  Builder builder)
{
  Built built;
  built.name = name;
  const auto start = std::chrono::steady_clock::now();
  auto trajectory = builder();
  const auto stop = std::chrono::steady_clock::now();
  built.compute_ms = std::chrono::duration<double, std::milli>(stop - start).count();
  built.metrics = robot_trajectory::evaluateTrajectory(*trajectory, 0.01);
  std::string error;
  built.csv_ok = robot_trajectory::writeTrajectoryCsv(csv_path, *trajectory, 0.01, error);
  if (!built.csv_ok) {
    std::cerr << name << " csv failed: " << error << "\n";
  }
  return built;
}

void printRow(const Built & built)
{
  std::cout << built.name
            << " compute_ms=" << built.compute_ms
            << " duration_s=" << built.metrics.duration
            << " jerk=" << built.metrics.jerk_cost
            << " max_vel=" << built.metrics.max_velocity
            << " max_acc=" << built.metrics.max_acceleration
            << " max_jerk=" << built.metrics.max_jerk
            << " continuous=" << (built.metrics.continuity.ok ? "true" : "false")
            << " pos_gap=" << built.metrics.continuity.position_gap
            << " vel_gap=" << built.metrics.continuity.velocity_gap
            << " acc_gap=" << built.metrics.continuity.acceleration_gap
            << "\n";
}

}  // namespace

int main()
{
  std::vector<Eigen::VectorXd> waypoints;
  Eigen::VectorXd start(6);
  Eigen::VectorXd mid(6);
  Eigen::VectorXd goal(6);
  start << 0.3, -1.0, 1.2, -1.5, 1.2, 0.5;
  mid << 0.0, -1.2, 1.4, -1.3, 0.9, 0.2;
  goal << -0.4, -0.8, 0.9, -1.0, 1.1, -0.3;
  waypoints.push_back(start);
  waypoints.push_back(mid);
  waypoints.push_back(goal);
  const std::vector<double> equal_times{1.5, 1.5};
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  const std::string root = "/home/parallels/robot_motion_ws/results";

  const Built quintic = run("quintic", root + "/phase4_quintic.csv", [&]() {
      auto trajectory = std::make_shared<robot_trajectory::JointQuinticTrajectory>();
      std::string error;
      if (!trajectory->generate(waypoints, equal_times, zero, zero, zero, zero, error)) {
        throw std::runtime_error(error);
      }
      return trajectory;
    });
  const Built jerk = run("minimum_jerk", root + "/phase5_minimum_jerk.csv", [&]() {
      auto trajectory = std::make_shared<robot_trajectory::MinimumJerkTrajectory>();
      std::string error;
      if (!trajectory->generate(waypoints, equal_times, zero, zero, zero, zero, error)) {
        throw std::runtime_error(error);
      }
      return trajectory;
    });
  const Built minco = run("minco", root + "/phase6_minco.csv", [&]() {
      auto trajectory = std::make_shared<robot_trajectory::MincoTrajectory>();
      std::string error;
      robot_trajectory::MincoOptions options;
      if (!trajectory->optimizeTimeAllocation(waypoints, 3.0, zero, zero, zero, zero, options, error)) {
        throw std::runtime_error(error);
      }
      std::cout << "minco_iterations=" << trajectory->iterations() << " times=";
      for (double duration : trajectory->segmentDurations()) {
        std::cout << duration << " ";
      }
      std::cout << "\n";
      return trajectory;
    });

  printRow(quintic);
  printRow(jerk);
  printRow(minco);
  if (!quintic.csv_ok || !jerk.csv_ok || !minco.csv_ok) {
    return 1;
  }
  if (!(jerk.metrics.jerk_cost <= quintic.metrics.jerk_cost &&
    minco.metrics.jerk_cost <= jerk.metrics.jerk_cost + 1e-6))
  {
    std::cerr << "cost ordering failed\n";
    return 1;
  }
  std::cout << "Acceptance: quintic, minimum jerk and MINCO passed\n";
  return 0;
}
