#include <cmath>

#include "gtest/gtest.h"
#include "robot_trajectory_optimization/minco.hpp"
#include "robot_trajectory_optimization/minimum_jerk.hpp"
#include "robot_trajectory_optimization/quintic_trajectory.hpp"

namespace
{

Eigen::VectorXd q(double value)
{
  Eigen::VectorXd out(1);
  out << value;
  return out;
}

std::vector<Eigen::VectorXd> scalarWaypoints(std::initializer_list<double> values)
{
  std::vector<Eigen::VectorXd> waypoints;
  for (double value : values) {
    waypoints.push_back(q(value));
  }
  return waypoints;
}

}  // namespace

TEST(Quintic, RestToRestMatchesClosedForm)
{
  robot_trajectory::QuinticTrajectory trajectory;
  std::string error;
  ASSERT_TRUE(trajectory.generate({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 1.0, error)) << error;
  EXPECT_NEAR(trajectory.coefficients()[3], 10.0, 1e-12);
  EXPECT_NEAR(trajectory.coefficients()[4], -15.0, 1e-12);
  EXPECT_NEAR(trajectory.coefficients()[5], 6.0, 1e-12);
  EXPECT_NEAR(trajectory.getPosition(0.0), 0.0, 1e-12);
  EXPECT_NEAR(trajectory.getPosition(1.0), 1.0, 1e-12);
  EXPECT_NEAR(trajectory.getPosition(0.5), 0.5, 1e-12);
  EXPECT_NEAR(trajectory.getVelocity(0.0), 0.0, 1e-12);
  EXPECT_NEAR(trajectory.getVelocity(1.0), 0.0, 1e-12);
  EXPECT_NEAR(trajectory.getAcceleration(0.0), 0.0, 1e-12);
  EXPECT_NEAR(trajectory.getAcceleration(1.0), 0.0, 1e-12);
}

TEST(Quintic, RejectsBadDurationAndMatchesNonzeroBoundary)
{
  robot_trajectory::QuinticTrajectory trajectory;
  std::string error;
  EXPECT_FALSE(trajectory.generate({0, 0, 0}, {1, 0, 0}, 0.0, error));
  EXPECT_FALSE(trajectory.generate({0, NAN, 0}, {1, 0, 0}, 1.0, error));
  ASSERT_TRUE(trajectory.generate({0.2, 0.5, -0.25}, {1.2, -0.3, 0.4}, 2.0, error)) << error;
  EXPECT_NEAR(trajectory.getPosition(0.0), 0.2, 1e-9);
  EXPECT_NEAR(trajectory.getPosition(2.0), 1.2, 1e-9);
  EXPECT_NEAR(trajectory.getVelocity(0.0), 0.5, 1e-9);
  EXPECT_NEAR(trajectory.getVelocity(2.0), -0.3, 1e-9);
  EXPECT_NEAR(trajectory.getAcceleration(0.0), -0.25, 1e-9);
  EXPECT_NEAR(trajectory.getAcceleration(2.0), 0.4, 1e-9);
}

TEST(JointQuintic, StopsAtIntermediateWaypoint)
{
  robot_trajectory::JointQuinticTrajectory trajectory;
  std::string error;
  const auto waypoints = scalarWaypoints({0.0, 1.0, 0.5});
  ASSERT_TRUE(trajectory.generate(
      waypoints, {1.0, 2.0}, q(0.0), q(0.0), q(0.0), q(0.0), error)) << error;
  EXPECT_NEAR(trajectory.position(1.0)[0], 1.0, 1e-9);
  EXPECT_NEAR(trajectory.velocity(1.0)[0], 0.0, 1e-8);
  EXPECT_NEAR(trajectory.acceleration(1.0)[0], 0.0, 1e-7);
  const auto report = robot_trajectory::checkContinuity(trajectory);
  EXPECT_TRUE(report.ok);
  EXPECT_NEAR(trajectory.totalDuration(), 3.0, 1e-12);
}

TEST(MinimumJerk, SingleSegmentMatchesQuinticAndMultiSegmentCostsLess)
{
  const auto waypoints = scalarWaypoints({0.0, 1.0, 0.25});
  const std::vector<double> times{1.0, 1.0};
  const Eigen::VectorXd zero = q(0.0);
  robot_trajectory::JointQuinticTrajectory quintic;
  robot_trajectory::MinimumJerkTrajectory jerk;
  std::string error;
  ASSERT_TRUE(quintic.generate(waypoints, times, zero, zero, zero, zero, error)) << error;
  ASSERT_TRUE(jerk.generate(waypoints, times, zero, zero, zero, zero, error)) << error;
  EXPECT_LT(jerk.jerkCost(), quintic.jerkCost() - 1e-6);
  const auto report = robot_trajectory::checkContinuity(jerk);
  EXPECT_TRUE(report.ok) << report.position_gap << " " << report.velocity_gap << " " << report.acceleration_gap;
  EXPECT_NEAR(jerk.position(0.0)[0], 0.0, 1e-8);
  EXPECT_NEAR(jerk.position(1.0)[0], 1.0, 1e-6);
  EXPECT_NEAR(jerk.position(2.0)[0], 0.25, 1e-6);
  EXPECT_NEAR(jerk.velocity(0.0)[0], 0.0, 1e-6);
  EXPECT_NEAR(jerk.velocity(2.0)[0], 0.0, 1e-6);
  EXPECT_GT(std::abs(jerk.velocity(1.0)[0]), 1e-3);

  robot_trajectory::MinimumJerkTrajectory one;
  ASSERT_TRUE(one.generate(scalarWaypoints({0.0, 1.0}), {1.5}, q(0.2), q(-0.1), q(-0.4), q(0.3), error));
  robot_trajectory::QuinticTrajectory scalar;
  ASSERT_TRUE(scalar.generate({0.0, 0.2, -0.1}, {1.0, -0.4, 0.3}, 1.5, error));
  EXPECT_NEAR(one.jerkCost(), scalar.jerkCost(), 1e-6);
  EXPECT_NEAR(one.position(0.4)[0], scalar.getPosition(0.4), 1e-6);
}

TEST(Minco, FixedTimeMatchesMinimumJerkAndAllocationDoesNotIncreaseCost)
{
  const auto waypoints = scalarWaypoints({0.0, 0.2, 1.4, 0.6});
  const std::vector<double> equal{1.0, 1.0, 1.0};
  const Eigen::VectorXd zero = q(0.0);
  robot_trajectory::MinimumJerkTrajectory jerk;
  robot_trajectory::MincoTrajectory fixed;
  robot_trajectory::MincoTrajectory optimized;
  std::string error;
  ASSERT_TRUE(jerk.generate(waypoints, equal, zero, zero, zero, zero, error)) << error;
  ASSERT_TRUE(fixed.generateFixedTime(waypoints, equal, zero, zero, zero, zero, error)) << error;
  EXPECT_NEAR(fixed.jerkCost(), jerk.jerkCost(), 1e-6);

  const Eigen::VectorXd gradient = fixed.timeGradient();
  std::vector<double> bumped = equal;
  bumped[0] += 1e-4;
  bumped[2] -= 1e-4;
  robot_trajectory::MinimumJerkTrajectory shifted;
  ASSERT_TRUE(shifted.generate(waypoints, bumped, zero, zero, zero, zero, error));
  const double predicted = jerk.jerkCost() + gradient[0] * 1e-4 + gradient[2] * (-1e-4);
  EXPECT_NEAR(shifted.jerkCost(), predicted, 1e-3);

  robot_trajectory::MincoOptions options;
  ASSERT_TRUE(optimized.optimizeTimeAllocation(waypoints, 3.0, zero, zero, zero, zero, options, error)) << error;
  EXPECT_NEAR(optimized.totalDuration(), 3.0, 1e-9);
  EXPECT_LE(optimized.jerkCost(), jerk.jerkCost() + 1e-6);
  EXPECT_TRUE(robot_trajectory::checkContinuity(optimized).ok);
}

TEST(JointQuintic, SixJointsStaySynchronized)
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
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  robot_trajectory::JointQuinticTrajectory trajectory;
  std::string error;
  ASSERT_TRUE(trajectory.generate(waypoints, {1.5, 1.5}, zero, zero, zero, zero, error)) << error;
  EXPECT_EQ(trajectory.dof(), 6);
  EXPECT_NEAR(trajectory.totalDuration(), 3.0, 1e-12);
  EXPECT_LT((trajectory.position(0.0) - start).norm(), 1e-9);
  EXPECT_LT((trajectory.position(1.5) - mid).norm(), 1e-8);
  EXPECT_LT((trajectory.position(3.0) - goal).norm(), 1e-8);
  EXPECT_LT(trajectory.velocity(1.5).norm(), 1e-7);
}
