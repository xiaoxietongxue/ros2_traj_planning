#pragma once

#include <string>
#include <vector>

#include "robot_trajectory_optimization/quintic_trajectory.hpp"

namespace robot_trajectory
{

// 固定时间的多段 minimum jerk。
// 路径点位置、总段时间、起终点速度和加速度固定。
// 中间路径点的速度和加速度是自由变量，由 jerk 平方积分最小来决定。
// 相邻段在位置、速度、加速度上连续。不要求在中间点停住。
class MinimumJerkTrajectory : public JointTrajectory
{
public:
  bool generate(
    const std::vector<Eigen::VectorXd> & waypoints,
    const std::vector<double> & segment_durations,
    const Eigen::VectorXd & start_velocity,
    const Eigen::VectorXd & start_acceleration,
    const Eigen::VectorXd & end_velocity,
    const Eigen::VectorXd & end_acceleration,
    std::string & error);

  int dof() const override { return dof_; }
  double totalDuration() const override { return total_duration_; }
  std::vector<double> segmentDurations() const override { return segment_durations_; }
  Eigen::VectorXd position(double t) const override;
  Eigen::VectorXd velocity(double t) const override;
  Eigen::VectorXd acceleration(double t) const override;
  Eigen::VectorXd jerk(double t) const override;
  double jerkCost() const override;

  // 包络定理：在最优中间速度/加速度处，代价对段时间的偏导数。
  Eigen::VectorXd timeGradient() const;

private:
  struct Piece
  {
    std::vector<QuinticTrajectory> joints;
  };

  const Piece & pieceAt(double t, double & local_time) const;
  Eigen::VectorXd evaluate(double t, double (QuinticTrajectory::*fn)(double) const) const;

  int dof_{0};
  double total_duration_{0.0};
  std::vector<double> segment_durations_;
  std::vector<Piece> pieces_;
};

}  // namespace robot_trajectory
