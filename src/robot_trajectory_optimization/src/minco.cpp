#include "robot_trajectory_optimization/minco.hpp"

#include <cmath>

namespace robot_trajectory
{
namespace
{

std::vector<double> softmaxDurations(const Eigen::VectorXd & unconstrained, double total_time)
{
  const double max_coeff = unconstrained.maxCoeff();
  double sum = 0.0;
  std::vector<double> weights(static_cast<std::size_t>(unconstrained.size()), 0.0);
  for (Eigen::Index i = 0; i < unconstrained.size(); ++i) {
    weights[static_cast<std::size_t>(i)] = std::exp(unconstrained[i] - max_coeff);
    sum += weights[static_cast<std::size_t>(i)];
  }
  std::vector<double> durations = weights;
  for (double & duration : durations) {
    duration = total_time * duration / sum;
  }
  return durations;
}

Eigen::VectorXd allocationGradient(
  const Eigen::VectorXd & unconstrained,
  const Eigen::VectorXd & time_gradient,
  double total_time)
{
  const auto durations = softmaxDurations(unconstrained, total_time);
  const int n = static_cast<int>(durations.size());
  Eigen::VectorXd softmax(n);
  for (int i = 0; i < n; ++i) {
    softmax[i] = durations[static_cast<std::size_t>(i)] / total_time;
  }
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n);
  for (int k = 0; k < n; ++k) {
    for (int i = 0; i < n; ++i) {
      const double jacobian = total_time * softmax[i] * ((i == k ? 1.0 : 0.0) - softmax[k]);
      gradient[k] += time_gradient[i] * jacobian;
    }
  }
  return gradient;
}

}  // namespace

bool MincoTrajectory::generateFixedTime(
  const std::vector<Eigen::VectorXd> & waypoints,
  const std::vector<double> & segment_durations,
  const Eigen::VectorXd & start_velocity,
  const Eigen::VectorXd & start_acceleration,
  const Eigen::VectorXd & end_velocity,
  const Eigen::VectorXd & end_acceleration,
  std::string & error)
{
  iterations_ = 0;
  return inner_.generate(
    waypoints, segment_durations, start_velocity, start_acceleration, end_velocity, end_acceleration, error);
}

bool MincoTrajectory::optimizeTimeAllocation(
  const std::vector<Eigen::VectorXd> & waypoints,
  double total_time,
  const Eigen::VectorXd & start_velocity,
  const Eigen::VectorXd & start_acceleration,
  const Eigen::VectorXd & end_velocity,
  const Eigen::VectorXd & end_acceleration,
  const MincoOptions & options,
  std::string & error)
{
  if (!(total_time > 0.0) || waypoints.size() < 2) {
    error = "MINCO needs a positive total time and at least two waypoints";
    return false;
  }
  const int segments = static_cast<int>(waypoints.size()) - 1;
  Eigen::VectorXd unconstrained = Eigen::VectorXd::Zero(segments);
  iterations_ = 0;
  double step = options.initial_step;

  std::vector<double> durations = softmaxDurations(unconstrained, total_time);
  if (!inner_.generate(
      waypoints, durations, start_velocity, start_acceleration, end_velocity, end_acceleration, error))
  {
    return false;
  }
  double current_cost = inner_.jerkCost();

  for (int iter = 0; iter < options.max_iterations; ++iter) {
    ++iterations_;
    const Eigen::VectorXd gradient = allocationGradient(unconstrained, inner_.timeGradient(), total_time);
    if (gradient.norm() < options.gradient_tolerance) {
      error.clear();
      return true;
    }
    bool accepted = false;
    double trial_step = step;
    for (int attempt = 0; attempt < 8; ++attempt) {
      const Eigen::VectorXd candidate = unconstrained - trial_step * gradient;
      const auto trial_durations = softmaxDurations(candidate, total_time);
      MinimumJerkTrajectory trial;
      std::string trial_error;
      if (!trial.generate(
          waypoints, trial_durations, start_velocity, start_acceleration,
          end_velocity, end_acceleration, trial_error))
      {
        trial_step *= 0.5;
        continue;
      }
      if (trial.jerkCost() <= current_cost) {
        unconstrained = candidate;
        durations = trial_durations;
        current_cost = trial.jerkCost();
        inner_ = std::move(trial);
        step = std::min(2.0, trial_step * 1.2);
        accepted = true;
        break;
      }
      trial_step *= 0.5;
    }
    if (!accepted) {
      error.clear();
      return true;
    }
  }
  error.clear();
  return true;
}

}  // namespace robot_trajectory
