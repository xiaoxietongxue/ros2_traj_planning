#include "robot_trajectory_optimization/quintic_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace robot_trajectory
{
namespace
{

bool finiteState(const ScalarState & state)
{
  return std::isfinite(state.position) && std::isfinite(state.velocity) &&
         std::isfinite(state.acceleration);
}

bool sameDimension(const std::vector<Eigen::VectorXd> & waypoints, std::string & error)
{
  if (waypoints.size() < 2) {
    error = "need at least two waypoints";
    return false;
  }
  const Eigen::Index n = waypoints.front().size();
  if (n < 1) {
    error = "waypoint dimension must be positive";
    return false;
  }
  for (const auto & waypoint : waypoints) {
    if (waypoint.size() != n || !waypoint.allFinite()) {
      error = "waypoints must be finite and share one dimension";
      return false;
    }
  }
  return true;
}

}  // namespace

bool QuinticTrajectory::generate(
  const ScalarState & start, const ScalarState & end, double duration, std::string & error)
{
  valid_ = false;
  if (!(duration > 0.0) || !std::isfinite(duration)) {
    error = "duration must be positive and finite";
    return false;
  }
  if (!finiteState(start) || !finiteState(end)) {
    error = "boundary state contains NaN or Inf";
    return false;
  }

  const double T = duration;
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;
  const double q0 = start.position;
  const double v0 = start.velocity;
  const double a0 = start.acceleration;
  const double qf = end.position;
  const double vf = end.velocity;
  const double af = end.acceleration;

  a_[0] = q0;
  a_[1] = v0;
  a_[2] = 0.5 * a0;
  a_[3] = (20.0 * qf - 20.0 * q0 - (8.0 * vf + 12.0 * v0) * T - (3.0 * a0 - af) * T2) / (2.0 * T3);
  a_[4] = (30.0 * q0 - 30.0 * qf + (14.0 * vf + 16.0 * v0) * T + (3.0 * a0 - 2.0 * af) * T2) / (2.0 * T4);
  a_[5] = (12.0 * qf - 12.0 * q0 - (6.0 * vf + 6.0 * v0) * T - (a0 - af) * T2) / (2.0 * T5);
  duration_ = duration;
  valid_ = true;
  error.clear();
  return true;
}

double QuinticTrajectory::clampedTime(double t) const
{
  return std::clamp(t, 0.0, duration_);
}

double QuinticTrajectory::getPosition(double t) const
{
  t = clampedTime(t);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  return a_[0] + a_[1] * t + a_[2] * t2 + a_[3] * t3 + a_[4] * t4 + a_[5] * t5;
}

double QuinticTrajectory::getVelocity(double t) const
{
  t = clampedTime(t);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  return a_[1] + 2.0 * a_[2] * t + 3.0 * a_[3] * t2 + 4.0 * a_[4] * t3 + 5.0 * a_[5] * t4;
}

double QuinticTrajectory::getAcceleration(double t) const
{
  t = clampedTime(t);
  const double t2 = t * t;
  const double t3 = t2 * t;
  return 2.0 * a_[2] + 6.0 * a_[3] * t + 12.0 * a_[4] * t2 + 20.0 * a_[5] * t3;
}

double QuinticTrajectory::getJerk(double t) const
{
  t = clampedTime(t);
  const double t2 = t * t;
  return 6.0 * a_[3] + 24.0 * a_[4] * t + 60.0 * a_[5] * t2;
}

double QuinticTrajectory::jerkCost() const
{
  const double T = duration_;
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;
  const double a3 = a_[3];
  const double a4 = a_[4];
  const double a5 = a_[5];
  return 36.0 * a3 * a3 * T +
         144.0 * a3 * a4 * T2 +
         240.0 * a3 * a5 * T3 +
         192.0 * a4 * a4 * T3 +
         720.0 * a4 * a5 * T4 +
         720.0 * a5 * a5 * T5;
}

std::vector<TrajectorySample> sampleTrajectory(const JointTrajectory & trajectory, double dt)
{
  std::vector<TrajectorySample> samples;
  if (!(dt > 0.0) || trajectory.dof() < 1) {
    return samples;
  }
  const double duration = trajectory.totalDuration();
  for (double t = 0.0; t < duration - 0.5 * dt; t += dt) {
    TrajectorySample sample;
    sample.time = t;
    sample.position = trajectory.position(t);
    sample.velocity = trajectory.velocity(t);
    sample.acceleration = trajectory.acceleration(t);
    sample.jerk = trajectory.jerk(t);
    samples.push_back(std::move(sample));
  }
  TrajectorySample last;
  last.time = duration;
  last.position = trajectory.position(duration);
  last.velocity = trajectory.velocity(duration);
  last.acceleration = trajectory.acceleration(duration);
  last.jerk = trajectory.jerk(duration);
  samples.push_back(std::move(last));
  return samples;
}

ContinuityReport checkContinuity(const JointTrajectory & trajectory, double epsilon)
{
  ContinuityReport report;
  const auto durations = trajectory.segmentDurations();
  double knot = 0.0;
  for (std::size_t i = 0; i + 1 < durations.size(); ++i) {
    knot += durations[i];
    const double left = std::max(0.0, knot - epsilon);
    const double right = std::min(trajectory.totalDuration(), knot + epsilon);
    report.position_gap = std::max(report.position_gap, (trajectory.position(left) - trajectory.position(right)).norm());
    report.velocity_gap = std::max(report.velocity_gap, (trajectory.velocity(left) - trajectory.velocity(right)).norm());
    report.acceleration_gap = std::max(
      report.acceleration_gap, (trajectory.acceleration(left) - trajectory.acceleration(right)).norm());
  }
  constexpr double kTol = 1e-5;
  report.ok = report.position_gap < kTol && report.velocity_gap < kTol && report.acceleration_gap < kTol;
  return report;
}

TrajectoryMetrics evaluateTrajectory(const JointTrajectory & trajectory, double sample_dt)
{
  TrajectoryMetrics metrics;
  metrics.duration = trajectory.totalDuration();
  metrics.jerk_cost = trajectory.jerkCost();
  metrics.continuity = checkContinuity(trajectory);
  const auto samples = sampleTrajectory(trajectory, sample_dt);
  for (const auto & sample : samples) {
    metrics.max_velocity = std::max(metrics.max_velocity, sample.velocity.cwiseAbs().maxCoeff());
    metrics.max_acceleration = std::max(metrics.max_acceleration, sample.acceleration.cwiseAbs().maxCoeff());
    metrics.max_jerk = std::max(metrics.max_jerk, sample.jerk.cwiseAbs().maxCoeff());
  }
  return metrics;
}

bool writeTrajectoryCsv(
  const std::string & path, const JointTrajectory & trajectory, double dt, std::string & error)
{
  const auto samples = sampleTrajectory(trajectory, dt);
  if (samples.empty()) {
    error = "trajectory sample is empty";
    return false;
  }
  std::ofstream out(path);
  if (!out) {
    error = "cannot open " + path;
    return false;
  }
  const int dof = trajectory.dof();
  out << "time";
  for (int j = 0; j < dof; ++j) out << ",pos_" << j;
  for (int j = 0; j < dof; ++j) out << ",vel_" << j;
  for (int j = 0; j < dof; ++j) out << ",acc_" << j;
  for (int j = 0; j < dof; ++j) out << ",jerk_" << j;
  out << "\n";
  out.setf(std::ios::fixed);
  out.precision(9);
  for (const auto & sample : samples) {
    out << sample.time;
    for (int j = 0; j < dof; ++j) out << "," << sample.position[j];
    for (int j = 0; j < dof; ++j) out << "," << sample.velocity[j];
    for (int j = 0; j < dof; ++j) out << "," << sample.acceleration[j];
    for (int j = 0; j < dof; ++j) out << "," << sample.jerk[j];
    out << "\n";
  }
  error.clear();
  return true;
}

bool JointQuinticTrajectory::generate(
  const std::vector<Eigen::VectorXd> & waypoints,
  const std::vector<double> & segment_durations,
  const Eigen::VectorXd & start_velocity,
  const Eigen::VectorXd & start_acceleration,
  const Eigen::VectorXd & end_velocity,
  const Eigen::VectorXd & end_acceleration,
  std::string & error)
{
  pieces_.clear();
  segment_durations_.clear();
  total_duration_ = 0.0;
  dof_ = 0;
  if (!sameDimension(waypoints, error)) {
    return false;
  }
  if (segment_durations.size() != waypoints.size() - 1) {
    error = "segment duration count must be waypoints - 1";
    return false;
  }
  const Eigen::Index dof = waypoints.front().size();
  if (start_velocity.size() != dof || start_acceleration.size() != dof ||
    end_velocity.size() != dof || end_acceleration.size() != dof ||
    !start_velocity.allFinite() || !start_acceleration.allFinite() ||
    !end_velocity.allFinite() || !end_acceleration.allFinite())
  {
    error = "boundary velocity and acceleration must match the waypoint dimension";
    return false;
  }
  for (double duration : segment_durations) {
    if (!(duration > 0.0) || !std::isfinite(duration)) {
      error = "every segment duration must be positive";
      return false;
    }
  }

  dof_ = static_cast<int>(dof);
  segment_durations_ = segment_durations;
  pieces_.resize(segment_durations.size());
  for (std::size_t seg = 0; seg < segment_durations.size(); ++seg) {
    pieces_[seg].joints.resize(static_cast<std::size_t>(dof));
    for (Eigen::Index joint = 0; joint < dof; ++joint) {
      ScalarState start;
      ScalarState end;
      start.position = waypoints[seg][joint];
      end.position = waypoints[seg + 1][joint];
      // 中间路径点强制停住，这样每段的速度和加速度在拼接点两侧都是 0。
      start.velocity = (seg == 0) ? start_velocity[joint] : 0.0;
      start.acceleration = (seg == 0) ? start_acceleration[joint] : 0.0;
      end.velocity = (seg + 1 == segment_durations.size()) ? end_velocity[joint] : 0.0;
      end.acceleration = (seg + 1 == segment_durations.size()) ? end_acceleration[joint] : 0.0;
      if (!pieces_[seg].joints[static_cast<std::size_t>(joint)].generate(
          start, end, segment_durations[seg], error))
      {
        return false;
      }
    }
    total_duration_ += segment_durations[seg];
  }
  error.clear();
  return true;
}

const JointQuinticTrajectory::Piece & JointQuinticTrajectory::pieceAt(double t, double & local_time) const
{
  t = std::clamp(t, 0.0, total_duration_);
  double consumed = 0.0;
  for (std::size_t i = 0; i < pieces_.size(); ++i) {
    const double duration = segment_durations_[i];
    if (t <= consumed + duration || i + 1 == pieces_.size()) {
      local_time = std::clamp(t - consumed, 0.0, duration);
      return pieces_[i];
    }
    consumed += duration;
  }
  local_time = 0.0;
  return pieces_.back();
}

Eigen::VectorXd JointQuinticTrajectory::evaluate(
  double t, double (QuinticTrajectory::*fn)(double) const) const
{
  double local = 0.0;
  const Piece & piece = pieceAt(t, local);
  Eigen::VectorXd value(dof_);
  for (int joint = 0; joint < dof_; ++joint) {
    value[joint] = (piece.joints[static_cast<std::size_t>(joint)].*fn)(local);
  }
  return value;
}

Eigen::VectorXd JointQuinticTrajectory::position(double t) const
{
  return evaluate(t, &QuinticTrajectory::getPosition);
}

Eigen::VectorXd JointQuinticTrajectory::velocity(double t) const
{
  return evaluate(t, &QuinticTrajectory::getVelocity);
}

Eigen::VectorXd JointQuinticTrajectory::acceleration(double t) const
{
  return evaluate(t, &QuinticTrajectory::getAcceleration);
}

Eigen::VectorXd JointQuinticTrajectory::jerk(double t) const
{
  return evaluate(t, &QuinticTrajectory::getJerk);
}

double JointQuinticTrajectory::jerkCost() const
{
  double cost = 0.0;
  for (const auto & piece : pieces_) {
    for (const auto & joint : piece.joints) {
      cost += joint.jerkCost();
    }
  }
  return cost;
}

}  // namespace robot_trajectory
