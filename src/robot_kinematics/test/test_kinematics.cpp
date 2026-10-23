#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "robot_kinematics/arm_kinematics.hpp"

namespace
{

double orientationError(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  const double dot = std::min(1.0, std::abs(a.normalized().dot(b.normalized())));
  return 2.0 * std::acos(dot);
}

class KinematicsTest : public ::testing::Test
{
public:
  static void release()
  {
    kinematics_.reset();
    node_.reset();
  }

protected:
  static void SetUpTestSuite()
  {
    node_ = std::make_shared<rclcpp::Node>("kinematics_test");
    kinematics_ = robot_kinematics::ArmKinematics::createUr5e(node_);
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<robot_kinematics::ArmKinematics> kinematics_;
};

rclcpp::Node::SharedPtr KinematicsTest::node_;
std::shared_ptr<robot_kinematics::ArmKinematics> KinematicsTest::kinematics_;

Eigen::VectorXd regularPose()
{
  Eigen::VectorXd q(6);
  q << 0.3, -1.0, 1.2, -1.5, 1.2, 0.5;
  return q;
}

Eigen::VectorXd singularPose()
{
  Eigen::VectorXd q(6);
  q << 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0;
  return q;
}

TEST_F(KinematicsTest, modelHasSixJoints)
{
  ASSERT_NE(kinematics_, nullptr);
  EXPECT_EQ(kinematics_->dof(), 6);
  EXPECT_EQ(kinematics_->groupName(), "ur_manipulator");
  EXPECT_EQ(kinematics_->tipLink(), "tool0");
  EXPECT_EQ(kinematics_->jointNames().size(), 6u);
}

TEST_F(KinematicsTest, rejectsBadInput)
{
  std::string error;
  robot_kinematics::EndEffectorPose pose;
  EXPECT_FALSE(kinematics_->computeFK(Eigen::VectorXd::Zero(3), pose, error));
  EXPECT_NE(error.find("expected"), std::string::npos);

  Eigen::VectorXd nan_q = regularPose();
  nan_q[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(kinematics_->computeFK(nan_q, pose, error));
  EXPECT_NE(error.find("NaN"), std::string::npos);

  Eigen::VectorXd outside = regularPose();
  outside[0] = 10.0;
  EXPECT_FALSE(kinematics_->withinLimits(outside, error));
  EXPECT_NE(error.find("outside"), std::string::npos);
}

TEST_F(KinematicsTest, fkJacobianAndManipulability)
{
  robot_kinematics::KinematicsReport report;
  ASSERT_TRUE(kinematics_->evaluate(regularPose(), report));
  EXPECT_TRUE(report.pose.position.allFinite());
  EXPECT_EQ(report.jacobian.rows(), 6);
  EXPECT_EQ(report.jacobian.cols(), 6);
  EXPECT_GT(report.manipulability, robot_kinematics::kSingularityThreshold);
  EXPECT_FALSE(report.near_singularity);

  const double shifted = report.pose.position.x();
  Eigen::VectorXd moved = regularPose();
  moved[0] += 0.4;
  robot_kinematics::EndEffectorPose other;
  std::string error;
  ASSERT_TRUE(kinematics_->computeFK(moved, other, error));
  EXPECT_GT((other.position - report.pose.position).norm(), 1e-3);
  EXPECT_NE(other.position.x(), shifted);
}

TEST_F(KinematicsTest, fkIkConsistency)
{
  robot_kinematics::EndEffectorPose pose;
  std::string error;
  ASSERT_TRUE(kinematics_->computeFK(regularPose(), pose, error));

  Eigen::VectorXd solution;
  ASSERT_TRUE(kinematics_->computeIK(pose, regularPose(), solution, error, 0.2)) << error;

  robot_kinematics::EndEffectorPose recovered;
  ASSERT_TRUE(kinematics_->computeFK(solution, recovered, error));
  EXPECT_LT((recovered.position - pose.position).norm(), 1e-3);
  EXPECT_LT(orientationError(recovered.orientation, pose.orientation), 1e-2);
}

TEST_F(KinematicsTest, unreachablePoseFailsIk)
{
  robot_kinematics::EndEffectorPose pose;
  std::string error;
  ASSERT_TRUE(kinematics_->computeFK(regularPose(), pose, error));
  pose.position = Eigen::Vector3d(5.0, 5.0, 5.0);

  Eigen::VectorXd solution;
  EXPECT_FALSE(kinematics_->computeIK(pose, regularPose(), solution, error, 0.05));
  EXPECT_NE(error.find("no solution"), std::string::npos);
}

TEST_F(KinematicsTest, wristSingularityLowersManipulability)
{
  double regular = 0.0;
  double singular = 0.0;
  std::string error;
  ASSERT_TRUE(kinematics_->computeManipulability(regularPose(), regular, error));
  ASSERT_TRUE(kinematics_->computeManipulability(singularPose(), singular, error));
  EXPECT_LT(singular, regular * 0.2);
  EXPECT_LT(singular, robot_kinematics::kSingularityThreshold);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  KinematicsTest::release();
  rclcpp::shutdown();
  return result;
}
