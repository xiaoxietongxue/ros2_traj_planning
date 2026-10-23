#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_kinematics
{

struct EndEffectorPose
{
  std::string model_frame;
  std::string tip_link;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

struct KinematicsReport
{
  bool ok{false};
  std::string message;
  EndEffectorPose pose;
  Eigen::MatrixXd jacobian;
  double manipulability{0.0};
  bool within_joint_limits{false};
  bool near_singularity{false};
};

// Yoshikawa 可操作度低于该值时，视为靠近奇异位形。
// 该量纲混合了米和弧度，只适合在同一机器人、同一参考点上做相对比较。
constexpr double kSingularityThreshold = 1.0e-3;

class ArmKinematics
{
public:
  static std::shared_ptr<ArmKinematics> createUr5e(const rclcpp::Node::SharedPtr & node);

  int dof() const;
  moveit::core::RobotModelConstPtr robotModel() const;
  std::string groupName() const;
  std::string tipLink() const;
  std::string modelFrame() const;
  std::vector<std::string> jointNames() const;

  bool withinLimits(const Eigen::VectorXd & q, std::string & error, double margin = 1e-6) const;
  bool computeFK(const Eigen::VectorXd & q, EndEffectorPose & pose, std::string & error) const;
  bool computeIK(
    const EndEffectorPose & target,
    const Eigen::VectorXd & seed,
    Eigen::VectorXd & solution,
    std::string & error,
    double timeout_s = 0.2) const;
  bool computeJacobian(const Eigen::VectorXd & q, Eigen::MatrixXd & jacobian, std::string & error) const;
  bool computeManipulability(const Eigen::VectorXd & q, double & manipulability, std::string & error) const;
  bool evaluate(const Eigen::VectorXd & q, KinematicsReport & report) const;

private:
  ArmKinematics(
    const rclcpp::Node::SharedPtr & node,
    const std::string & urdf,
    const std::string & srdf);

  bool prepareState(const Eigen::VectorXd & q, std::string & error) const;
  static double manipulabilityOf(const Eigen::MatrixXd & jacobian);

  rclcpp::Node::SharedPtr node_;
  // RobotModelLoader 必须活着，否则运动学插件会被卸载。
  std::shared_ptr<class RobotModelHolder> holder_;
  std::string group_name_;
  std::string tip_link_;
};

}  // namespace robot_kinematics
