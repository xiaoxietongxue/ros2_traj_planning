#pragma once

#include <string>
#include <vector>

#include "robot_trajectory_optimization/minimum_jerk.hpp"

namespace robot_trajectory
{

struct MincoOptions
{
  int max_iterations{40};
  double initial_step{0.5};
  double gradient_tolerance{1e-3};
};

// 关节空间的 MINCO（s = 3，最小化 jerk）。
// 固定路径点和总时间时，优化变量只剩各段时间。
// 系数由块三对角的 minimum-jerk 线性系统恢复，复杂度随段数线性增长。
// 这不是“每段单独做五次插值”。中间速度不强制为 0，时间分配也会变。
class MincoTrajectory : public JointTrajectory
{
public:
  bool generateFixedTime(
    const std::vector<Eigen::VectorXd> & waypoints,
    const std::vector<double> & segment_durations,
    const Eigen::VectorXd & start_velocity,
    const Eigen::VectorXd & start_acceleration,
    const Eigen::VectorXd & end_velocity,
    const Eigen::VectorXd & end_acceleration,
    std::string & error);

  // 总时间固定，用 softmax 参数化各段时间，沿 jerk 代价的梯度下降。
  bool optimizeTimeAllocation(
    const std::vector<Eigen::VectorXd> & waypoints,
    double total_time,
    const Eigen::VectorXd & start_velocity,
    const Eigen::VectorXd & start_acceleration,
    const Eigen::VectorXd & end_velocity,
    const Eigen::VectorXd & end_acceleration,
    const MincoOptions & options,
    std::string & error);

  int dof() const override { return inner_.dof(); }
  double totalDuration() const override { return inner_.totalDuration(); }
  std::vector<double> segmentDurations() const override { return inner_.segmentDurations(); }
  Eigen::VectorXd position(double t) const override { return inner_.position(t); }
  Eigen::VectorXd velocity(double t) const override { return inner_.velocity(t); }
  Eigen::VectorXd acceleration(double t) const override { return inner_.acceleration(t); }
  Eigen::VectorXd jerk(double t) const override { return inner_.jerk(t); }
  double jerkCost() const override { return inner_.jerkCost(); }
  Eigen::VectorXd timeGradient() const { return inner_.timeGradient(); }
  int iterations() const { return iterations_; }

private:
  MinimumJerkTrajectory inner_;
  int iterations_{0};
};

}  // namespace robot_trajectory
