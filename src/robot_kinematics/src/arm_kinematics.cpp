#include "robot_kinematics/arm_kinematics.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <utility>

#include <Eigen/Eigenvalues>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace robot_kinematics
{
namespace
{

std::string readCommand(const std::string & command, int & status)
{
  std::string output;
  FILE * pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    status = -1;
    return output;
  }
  char buffer[512];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }
  status = pclose(pipe);
  if (status != -1) {
    status = WEXITSTATUS(status);
  }
  return output;
}

std::string loadXacro(const std::string & command)
{
  int status = 0;
  const std::string output = readCommand(command + " 2>/tmp/robot_kinematics_xacro.err", status);
  if (status != 0 || output.find("<robot") == std::string::npos) {
    std::string detail;
    int err_status = 0;
    detail = readCommand("cat /tmp/robot_kinematics_xacro.err", err_status);
    throw std::runtime_error("xacro failed (" + std::to_string(status) + "): " + command + "\n" + detail);
  }
  return output;
}

std::vector<double> toStdVector(const Eigen::VectorXd & q)
{
  return std::vector<double>(q.data(), q.data() + q.size());
}

}  // namespace

class RobotModelHolder
{
public:
  // loader 必须最后析构，否则运动学插件会在 RobotModel 仍持有求解器时被卸载。
  robot_model_loader::RobotModelLoaderPtr loader;
  moveit::core::RobotModelPtr model;
  std::unique_ptr<moveit::core::RobotState> state;
  const moveit::core::JointModelGroup * group{nullptr};
};

std::shared_ptr<ArmKinematics> ArmKinematics::createUr5e(const rclcpp::Node::SharedPtr & node)
{
  const std::string urdf_xacro =
    ament_index_cpp::get_package_share_directory("ur_description") + "/urdf/ur.urdf.xacro";
  const std::string srdf_xacro =
    ament_index_cpp::get_package_share_directory("ur_moveit_config") + "/srdf/ur.srdf.xacro";

  const std::string urdf = loadXacro(
    "xacro " + urdf_xacro +
    " ur_type:=ur5e name:=ur safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 tf_prefix:=");
  const std::string srdf = loadXacro("xacro " + srdf_xacro + " name:=ur");
  return std::shared_ptr<ArmKinematics>(new ArmKinematics(node, urdf, srdf));
}

ArmKinematics::ArmKinematics(
  const rclcpp::Node::SharedPtr & node,
  const std::string & urdf,
  const std::string & srdf)
: node_(node), group_name_("ur_manipulator"), tip_link_("tool0")
{
  auto declare_string = [this](const std::string & name, const std::string & value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<std::string>(name, value);
    } else {
      node_->set_parameter(rclcpp::Parameter(name, value));
    }
  };
  auto declare_double = [this](const std::string & name, double value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<double>(name, value);
    }
  };
  auto declare_int = [this](const std::string & name, int value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<int>(name, value);
    }
  };

  declare_string("robot_description", urdf);
  declare_string("robot_description_semantic", srdf);
  const std::string prefix = "robot_description_kinematics." + group_name_ + ".";
  declare_string(prefix + "kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  declare_double(prefix + "kinematics_solver_search_resolution", 0.005);
  declare_double(prefix + "kinematics_solver_timeout", 0.05);
  declare_int(prefix + "kinematics_solver_attempts", 3);

  holder_ = std::make_shared<RobotModelHolder>();
  holder_->loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
  holder_->model = holder_->loader->getModel();
  if (!holder_->model) {
    throw std::runtime_error("RobotModelLoader returned an empty UR5e model");
  }
  holder_->group = holder_->model->getJointModelGroup(group_name_);
  if (holder_->group == nullptr) {
    throw std::runtime_error("joint group not found: " + group_name_);
  }
  if (!holder_->group->hasLinkModel(tip_link_) && holder_->model->getLinkModel(tip_link_) == nullptr) {
    throw std::runtime_error("tip link not found: " + tip_link_);
  }
  if (!holder_->group->canSetStateFromIK(tip_link_)) {
    throw std::runtime_error("kinematics solver is not available for " + tip_link_);
  }
  holder_->state = std::make_unique<moveit::core::RobotState>(holder_->model);
  holder_->state->setToDefaultValues();
}

int ArmKinematics::dof() const
{
  return static_cast<int>(holder_->group->getVariableCount());
}

moveit::core::RobotModelConstPtr ArmKinematics::robotModel() const
{
  return holder_->model;
}

std::string ArmKinematics::groupName() const
{
  return group_name_;
}

std::string ArmKinematics::tipLink() const
{
  return tip_link_;
}

std::string ArmKinematics::modelFrame() const
{
  return holder_->model->getModelFrame();
}

std::vector<std::string> ArmKinematics::jointNames() const
{
  return holder_->group->getActiveJointModelNames();
}

bool ArmKinematics::withinLimits(const Eigen::VectorXd & q, std::string & error, double margin) const
{
  if (q.size() != dof()) {
    error = "expected " + std::to_string(dof()) + " joints, got " + std::to_string(q.size());
    return false;
  }
  if (!q.allFinite()) {
    error = "joint vector contains NaN or Inf";
    return false;
  }

  const auto names = jointNames();
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto * joint = holder_->group->getJointModel(names[i]);
    const auto & bounds = joint->getVariableBounds();
    if (bounds.empty() || !bounds[0].position_bounded_) {
      continue;
    }
    if (q[static_cast<Eigen::Index>(i)] < bounds[0].min_position_ - margin ||
      q[static_cast<Eigen::Index>(i)] > bounds[0].max_position_ + margin)
    {
      std::ostringstream text;
      text << names[i] << " = " << q[static_cast<Eigen::Index>(i)]
           << " is outside [" << bounds[0].min_position_ << ", " << bounds[0].max_position_ << "]";
      error = text.str();
      return false;
    }
  }
  error.clear();
  return true;
}

bool ArmKinematics::prepareState(const Eigen::VectorXd & q, std::string & error) const
{
  if (!withinLimits(q, error)) {
    return false;
  }
  holder_->state->setJointGroupPositions(holder_->group, toStdVector(q));
  holder_->state->update();
  return true;
}

bool ArmKinematics::computeFK(const Eigen::VectorXd & q, EndEffectorPose & pose, std::string & error) const
{
  if (!prepareState(q, error)) {
    return false;
  }
  const Eigen::Isometry3d transform = holder_->state->getGlobalLinkTransform(tip_link_);
  pose.model_frame = modelFrame();
  pose.tip_link = tip_link_;
  pose.position = transform.translation();
  pose.rotation = transform.rotation();
  pose.orientation = Eigen::Quaterniond(pose.rotation);
  pose.orientation.normalize();
  error.clear();
  return true;
}

bool ArmKinematics::computeIK(
  const EndEffectorPose & target,
  const Eigen::VectorXd & seed,
  Eigen::VectorXd & solution,
  std::string & error,
  double timeout_s) const
{
  if (!withinLimits(seed, error)) {
    return false;
  }
  if (!target.position.allFinite() || !target.orientation.coeffs().allFinite()) {
    error = "target pose contains NaN or Inf";
    return false;
  }
  if (target.orientation.norm() < 1e-8) {
    error = "target orientation quaternion has zero norm";
    return false;
  }
  if (!(timeout_s > 0.0)) {
    error = "IK timeout must be positive";
    return false;
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = target.orientation.normalized().toRotationMatrix();
  pose.translation() = target.position;

  holder_->state->setJointGroupPositions(holder_->group, toStdVector(seed));
  holder_->state->update();
  const bool solved = holder_->state->setFromIK(holder_->group, pose, tip_link_, timeout_s);
  if (!solved) {
    error = "IK solver returned no solution. The pose may be unreachable, or the seed is far from a solution.";
    return false;
  }
  if (!holder_->state->satisfiesBounds(holder_->group, 1e-4)) {
    error = "IK solution violates joint limits";
    return false;
  }

  std::vector<double> values;
  holder_->state->copyJointGroupPositions(holder_->group, values);
  solution = Eigen::Map<Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
  error.clear();
  return true;
}

bool ArmKinematics::computeJacobian(
  const Eigen::VectorXd & q, Eigen::MatrixXd & jacobian, std::string & error) const
{
  if (!prepareState(q, error)) {
    return false;
  }
  const Eigen::Vector3d reference = Eigen::Vector3d::Zero();
  if (!holder_->state->getJacobian(holder_->group, holder_->model->getLinkModel(tip_link_), reference, jacobian)) {
    error = "Jacobian computation failed";
    return false;
  }
  if (jacobian.rows() != 6 || jacobian.cols() != dof()) {
    error = "unexpected Jacobian size";
    return false;
  }
  error.clear();
  return true;
}

double ArmKinematics::manipulabilityOf(const Eigen::MatrixXd & jacobian)
{
  const Eigen::MatrixXd gram = jacobian * jacobian.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(gram);
  if (solver.info() != Eigen::Success) {
    return 0.0;
  }
  double determinant = 1.0;
  for (Eigen::Index i = 0; i < solver.eigenvalues().size(); ++i) {
    const double lambda = std::max(0.0, solver.eigenvalues()[i]);
    determinant *= lambda;
  }
  return std::sqrt(determinant);
}

bool ArmKinematics::computeManipulability(
  const Eigen::VectorXd & q, double & manipulability, std::string & error) const
{
  Eigen::MatrixXd jacobian;
  if (!computeJacobian(q, jacobian, error)) {
    return false;
  }
  manipulability = manipulabilityOf(jacobian);
  if (!std::isfinite(manipulability)) {
    error = "manipulability is not finite";
    return false;
  }
  error.clear();
  return true;
}

bool ArmKinematics::evaluate(const Eigen::VectorXd & q, KinematicsReport & report) const
{
  report = KinematicsReport();
  report.within_joint_limits = withinLimits(q, report.message);
  if (!report.within_joint_limits) {
    return false;
  }
  if (!computeFK(q, report.pose, report.message)) {
    return false;
  }
  if (!computeJacobian(q, report.jacobian, report.message)) {
    return false;
  }
  report.manipulability = manipulabilityOf(report.jacobian);
  report.near_singularity = report.manipulability < kSingularityThreshold;
  report.ok = true;
  report.message.clear();
  return true;
}

}  // namespace robot_kinematics
