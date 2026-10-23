#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace robot_trajectory
{

struct ScalarState
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

// 单关节、单段五次多项式。边界条件是两端的位置、速度、加速度。
class QuinticTrajectory
{
public:
  bool generate(
    const ScalarState & start, const ScalarState & end, double duration, std::string & error);

  bool valid() const { return valid_; }
  double duration() const { return duration_; }
  const std::array<double, 6> & coefficients() const { return a_; }

  double getPosition(double t) const;
  double getVelocity(double t) const;
  double getAcceleration(double t) const;
  double getJerk(double t) const;
  double jerkCost() const;

private:
  double clampedTime(double t) const;

  double duration_{0.0};
  std::array<double, 6> a_{{0, 0, 0, 0, 0, 0}};
  bool valid_{false};
};

struct ContinuityReport
{
  double position_gap{0.0};
  double velocity_gap{0.0};
  double acceleration_gap{0.0};
  bool ok{false};
};

struct TrajectoryMetrics
{
  double duration{0.0};
  double jerk_cost{0.0};
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double max_jerk{0.0};
  ContinuityReport continuity;
};

// 多关节轨迹的统一读接口。t 从 0 到 totalDuration()。
class JointTrajectory
{
public:
  virtual ~JointTrajectory() = default;
  virtual int dof() const = 0;
  virtual double totalDuration() const = 0;
  virtual std::vector<double> segmentDurations() const = 0;
  virtual Eigen::VectorXd position(double t) const = 0;
  virtual Eigen::VectorXd velocity(double t) const = 0;
  virtual Eigen::VectorXd acceleration(double t) const = 0;
  virtual Eigen::VectorXd jerk(double t) const = 0;
  virtual double jerkCost() const = 0;
};

struct TrajectorySample
{
  double time{0.0};
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd jerk;
};

std::vector<TrajectorySample> sampleTrajectory(const JointTrajectory & trajectory, double dt);
ContinuityReport checkContinuity(const JointTrajectory & trajectory, double epsilon = 1e-7);
TrajectoryMetrics evaluateTrajectory(const JointTrajectory & trajectory, double sample_dt);
bool writeTrajectoryCsv(
  const std::string & path, const JointTrajectory & trajectory, double dt, std::string & error);

// 每个路径点速度和加速度都为 0 的同步五次多项式。关节同时起步、同时停止。
class JointQuinticTrajectory : public JointTrajectory
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
