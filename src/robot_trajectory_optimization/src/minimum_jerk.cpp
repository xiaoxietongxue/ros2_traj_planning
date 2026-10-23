#include "robot_trajectory_optimization/minimum_jerk.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>

namespace robot_trajectory
{
namespace
{

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

double segmentCost(double q0, double v0, double a0, double qf, double vf, double af, double duration)
{
  QuinticTrajectory piece;
  std::string error;
  ScalarState start{q0, v0, a0};
  ScalarState end{qf, vf, af};
  if (!piece.generate(start, end, duration, error)) {
    return std::numeric_limits<double>::infinity();
  }
  return piece.jerkCost();
}

// 块三对角 Thomas 算法。每个块是 2×2，对应一个中间路径点的 (速度, 加速度)。
bool solveBlockTridiagonal(
  const std::vector<Eigen::Matrix2d> & diag,
  const std::vector<Eigen::Matrix2d> & lower,
  const std::vector<Eigen::Matrix2d> & upper,
  const Eigen::VectorXd & rhs,
  Eigen::VectorXd & solution,
  std::string & error)
{
  const int n = static_cast<int>(diag.size());
  if (n == 0) {
    solution.resize(0);
    return true;
  }
  if (static_cast<int>(rhs.size()) != 2 * n ||
    static_cast<int>(lower.size()) != n - 1 ||
    static_cast<int>(upper.size()) != n - 1)
  {
    error = "block-tridiagonal size mismatch";
    return false;
  }

  std::vector<Eigen::Matrix2d> diagonal = diag;
  std::vector<Eigen::Matrix2d> carry_upper(static_cast<std::size_t>(std::max(0, n - 1)));
  std::vector<Eigen::Vector2d> carry_rhs(static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    Eigen::Matrix2d pivot = diagonal[static_cast<std::size_t>(i)];
    Eigen::Vector2d row_rhs = rhs.segment<2>(2 * i);
    if (i > 0) {
      pivot.noalias() -= lower[static_cast<std::size_t>(i - 1)] * carry_upper[static_cast<std::size_t>(i - 1)];
      row_rhs.noalias() -= lower[static_cast<std::size_t>(i - 1)] * carry_rhs[static_cast<std::size_t>(i - 1)];
    }
    Eigen::LDLT<Eigen::Matrix2d> factorization(pivot);
    if (factorization.info() != Eigen::Success || !pivot.allFinite()) {
      error = "minimum-jerk Hessian block is not positive definite";
      return false;
    }
    carry_rhs[static_cast<std::size_t>(i)] = factorization.solve(row_rhs);
    if (i + 1 < n) {
      carry_upper[static_cast<std::size_t>(i)] = factorization.solve(upper[static_cast<std::size_t>(i)]);
    }
  }

  solution.resize(2 * n);
  for (int i = n - 1; i >= 0; --i) {
    Eigen::Vector2d value = carry_rhs[static_cast<std::size_t>(i)];
    if (i + 1 < n) {
      value.noalias() -= carry_upper[static_cast<std::size_t>(i)] * solution.segment<2>(2 * (i + 1));
    }
    solution.segment<2>(2 * i) = value;
  }
  error.clear();
  return true;
}

}  // namespace

bool MinimumJerkTrajectory::generate(
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
  const int dof = static_cast<int>(waypoints.front().size());
  const int segments = static_cast<int>(segment_durations.size());
  const int interiors = segments - 1;
  if (start_velocity.size() != dof || start_acceleration.size() != dof ||
    end_velocity.size() != dof || end_acceleration.size() != dof)
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

  std::vector<Eigen::VectorXd> velocity(static_cast<std::size_t>(segments + 1), Eigen::VectorXd::Zero(dof));
  std::vector<Eigen::VectorXd> acceleration(static_cast<std::size_t>(segments + 1), Eigen::VectorXd::Zero(dof));
  velocity.front() = start_velocity;
  acceleration.front() = start_acceleration;
  velocity.back() = end_velocity;
  acceleration.back() = end_acceleration;

  const double h = 1e-4;
  for (int joint = 0; joint < dof; ++joint) {
    const int dim = 2 * interiors;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dim);

    for (int seg = 0; seg < segments; ++seg) {
      Eigen::Vector4d base;
      base << velocity[static_cast<std::size_t>(seg)][joint],
        acceleration[static_cast<std::size_t>(seg)][joint],
        velocity[static_cast<std::size_t>(seg + 1)][joint],
        acceleration[static_cast<std::size_t>(seg + 1)][joint];
      // 局部顺序：起点速度、起点加速度、终点速度、终点加速度。
      // 负的全局下标表示这个量被边界条件固定。
      int slot_global[4] = {-1, -1, -1, -1};
      if (seg > 0) {
        slot_global[0] = 2 * (seg - 1);
        slot_global[1] = 2 * (seg - 1) + 1;
      }
      if (seg + 1 < segments) {
        slot_global[2] = 2 * seg;
        slot_global[3] = 2 * seg + 1;
      }
      std::vector<int> free_slots;
      std::vector<int> free_global;
      for (int slot = 0; slot < 4; ++slot) {
        if (slot_global[slot] >= 0) {
          free_slots.push_back(slot);
          free_global.push_back(slot_global[slot]);
        }
      }
      if (free_slots.empty()) {
        continue;
      }
      const int k = static_cast<int>(free_slots.size());
      const double q0 = waypoints[static_cast<std::size_t>(seg)][joint];
      const double qf = waypoints[static_cast<std::size_t>(seg + 1)][joint];
      const double duration = segment_durations[static_cast<std::size_t>(seg)];

      auto evaluate = [&](const Eigen::VectorXd & delta) {
          Eigen::Vector4d state = base;
          for (int i = 0; i < k; ++i) {
            state[free_slots[static_cast<std::size_t>(i)]] += delta[i];
          }
          return segmentCost(q0, state[0], state[1], qf, state[2], state[3], duration);
        };

      Eigen::VectorXd local_gradient(k);
      Eigen::MatrixXd local_hessian(k, k);
      for (int i = 0; i < k; ++i) {
        Eigen::VectorXd step = Eigen::VectorXd::Zero(k);
        step[i] = h;
        local_gradient[i] = (evaluate(step) - evaluate(-step)) / (2.0 * h);
        for (int j = i; j < k; ++j) {
          Eigen::VectorXd pp = Eigen::VectorXd::Zero(k);
          Eigen::VectorXd pm = Eigen::VectorXd::Zero(k);
          Eigen::VectorXd mp = Eigen::VectorXd::Zero(k);
          Eigen::VectorXd mm = Eigen::VectorXd::Zero(k);
          pp[i] = h;
          pp[j] += h;
          pm[i] = h;
          pm[j] -= h;
          mp[i] = -h;
          mp[j] += h;
          mm[i] = -h;
          mm[j] -= h;
          const double second = (evaluate(pp) - evaluate(pm) - evaluate(mp) + evaluate(mm)) / (4.0 * h * h);
          local_hessian(i, j) = second;
          local_hessian(j, i) = second;
        }
      }
      for (int i = 0; i < k; ++i) {
        gradient[free_global[static_cast<std::size_t>(i)]] += local_gradient[i];
        for (int j = 0; j < k; ++j) {
          hessian(free_global[static_cast<std::size_t>(i)], free_global[static_cast<std::size_t>(j)]) +=
            local_hessian(i, j);
        }
      }
    }

    Eigen::VectorXd solved(dim);
    if (dim == 0) {
      solved.resize(0);
    } else {
      std::vector<Eigen::Matrix2d> diag(static_cast<std::size_t>(interiors), Eigen::Matrix2d::Zero());
      std::vector<Eigen::Matrix2d> lower;
      std::vector<Eigen::Matrix2d> upper;
      if (interiors > 1) {
        lower.resize(static_cast<std::size_t>(interiors - 1), Eigen::Matrix2d::Zero());
        upper.resize(static_cast<std::size_t>(interiors - 1), Eigen::Matrix2d::Zero());
      }
      for (int i = 0; i < interiors; ++i) {
        diag[static_cast<std::size_t>(i)] = hessian.block<2, 2>(2 * i, 2 * i);
        if (i + 1 < interiors) {
          upper[static_cast<std::size_t>(i)] = hessian.block<2, 2>(2 * i, 2 * (i + 1));
          lower[static_cast<std::size_t>(i)] = hessian.block<2, 2>(2 * (i + 1), 2 * i);
        }
      }
      if (!solveBlockTridiagonal(diag, lower, upper, -gradient, solved, error)) {
        return false;
      }
    }
    for (int point = 1; point <= interiors; ++point) {
      velocity[static_cast<std::size_t>(point)][joint] = solved[2 * (point - 1)];
      acceleration[static_cast<std::size_t>(point)][joint] = solved[2 * (point - 1) + 1];
    }
  }

  dof_ = dof;
  segment_durations_ = segment_durations;
  pieces_.resize(static_cast<std::size_t>(segments));
  for (int seg = 0; seg < segments; ++seg) {
    pieces_[static_cast<std::size_t>(seg)].joints.resize(static_cast<std::size_t>(dof));
    for (int joint = 0; joint < dof; ++joint) {
      ScalarState start{
        waypoints[static_cast<std::size_t>(seg)][joint],
        velocity[static_cast<std::size_t>(seg)][joint],
        acceleration[static_cast<std::size_t>(seg)][joint]};
      ScalarState end{
        waypoints[static_cast<std::size_t>(seg + 1)][joint],
        velocity[static_cast<std::size_t>(seg + 1)][joint],
        acceleration[static_cast<std::size_t>(seg + 1)][joint]};
      if (!pieces_[static_cast<std::size_t>(seg)].joints[static_cast<std::size_t>(joint)].generate(
          start, end, segment_durations[static_cast<std::size_t>(seg)], error))
      {
        return false;
      }
    }
    total_duration_ += segment_durations[static_cast<std::size_t>(seg)];
  }
  error.clear();
  return true;
}

const MinimumJerkTrajectory::Piece & MinimumJerkTrajectory::pieceAt(double t, double & local_time) const
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

Eigen::VectorXd MinimumJerkTrajectory::evaluate(
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

Eigen::VectorXd MinimumJerkTrajectory::position(double t) const
{
  return evaluate(t, &QuinticTrajectory::getPosition);
}

Eigen::VectorXd MinimumJerkTrajectory::velocity(double t) const
{
  return evaluate(t, &QuinticTrajectory::getVelocity);
}

Eigen::VectorXd MinimumJerkTrajectory::acceleration(double t) const
{
  return evaluate(t, &QuinticTrajectory::getAcceleration);
}

Eigen::VectorXd MinimumJerkTrajectory::jerk(double t) const
{
  return evaluate(t, &QuinticTrajectory::getJerk);
}

double MinimumJerkTrajectory::jerkCost() const
{
  double cost = 0.0;
  for (const auto & piece : pieces_) {
    for (const auto & joint : piece.joints) {
      cost += joint.jerkCost();
    }
  }
  return cost;
}

Eigen::VectorXd MinimumJerkTrajectory::timeGradient() const
{
  const double h = 1e-5;
  Eigen::VectorXd gradient(static_cast<Eigen::Index>(segment_durations_.size()));
  for (std::size_t seg = 0; seg < segment_durations_.size(); ++seg) {
    double derivative = 0.0;
    for (const auto & joint : pieces_[seg].joints) {
      const double T = joint.duration();
      const double q0 = joint.getPosition(0.0);
      const double v0 = joint.getVelocity(0.0);
      const double a0 = joint.getAcceleration(0.0);
      const double qf = joint.getPosition(T);
      const double vf = joint.getVelocity(T);
      const double af = joint.getAcceleration(T);
      derivative += (segmentCost(q0, v0, a0, qf, vf, af, T + h) -
        segmentCost(q0, v0, a0, qf, vf, af, std::max(1e-4, T - h))) /
        (2.0 * h);
    }
    gradient[static_cast<Eigen::Index>(seg)] = derivative;
  }
  return gradient;
}

}  // namespace robot_trajectory
