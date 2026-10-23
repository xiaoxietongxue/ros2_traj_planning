#include <iomanip>
#include <iostream>
#include <string>

#include "robot_kinematics/arm_kinematics.hpp"

namespace
{

void printVector(const std::string & title, const Eigen::VectorXd & values)
{
  std::cout << title << ":";
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    std::cout << " " << std::fixed << std::setprecision(4) << values[i];
  }
  std::cout << "\n";
}

void printReport(const std::string & title, const robot_kinematics::KinematicsReport & report)
{
  std::cout << "\n== " << title << " ==\n";
  std::cout << "frame: " << report.pose.model_frame << " -> " << report.pose.tip_link << "\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "position_m: " << report.pose.position.transpose() << "\n";
  std::cout << "quaternion_xyzw: " << report.pose.orientation.x() << " "
            << report.pose.orientation.y() << " "
            << report.pose.orientation.z() << " "
            << report.pose.orientation.w() << "\n";
  std::cout << std::setprecision(6);
  std::cout << "manipulability: " << report.manipulability
            << (report.near_singularity ? "  (near singularity)" : "  (away from threshold)") << "\n";
  std::cout << "jacobian_6xn:\n" << report.jacobian << "\n";
}

double orientationError(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b)
{
  const double dot = std::min(1.0, std::abs(a.normalized().dot(b.normalized())));
  return 2.0 * std::acos(dot);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kinematics_demo");

  std::shared_ptr<robot_kinematics::ArmKinematics> kinematics;
  const auto finish = [&](int code) {
    kinematics.reset();
    node.reset();
    rclcpp::shutdown();
    return code;
  };
  try {
    kinematics = robot_kinematics::ArmKinematics::createUr5e(node);
  } catch (const std::exception & ex) {
    std::cerr << "Failed to load UR5e kinematics: " << ex.what() << "\n";
    return finish(1);
  }

  const auto names = kinematics->jointNames();
  std::cout << "group: " << kinematics->groupName()
            << "  tip: " << kinematics->tipLink()
            << "  model_frame: " << kinematics->modelFrame()
            << "  dof: " << kinematics->dof() << "\n";
  std::cout << "joints:";
  for (const auto & name : names) {
    std::cout << " " << name;
  }
  std::cout << "\n";

  Eigen::VectorXd regular(6);
  regular << 0.3, -1.0, 1.2, -1.5, 1.2, 0.5;
  Eigen::VectorXd singular(6);
  singular << 0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0;

  robot_kinematics::KinematicsReport regular_report;
  robot_kinematics::KinematicsReport singular_report;
  if (!kinematics->evaluate(regular, regular_report) || !kinematics->evaluate(singular, singular_report)) {
    std::cerr << "evaluate failed: " << regular_report.message << " | " << singular_report.message << "\n";
    return finish(1);
  }
  printReport("regular configuration", regular_report);
  printReport("wrist-singular configuration (wrist_2 = 0)", singular_report);

  Eigen::VectorXd solution;
  std::string error;
  const bool ik_ok = kinematics->computeIK(regular_report.pose, regular, solution, error, 0.2);
  if (!ik_ok) {
    std::cerr << "IK failed: " << error << "\n";
    return finish(1);
  }
  robot_kinematics::EndEffectorPose recovered;
  if (!kinematics->computeFK(solution, recovered, error)) {
    std::cerr << "FK of IK solution failed: " << error << "\n";
    return finish(1);
  }

  const double position_error = (recovered.position - regular_report.pose.position).norm();
  const double angle_error = orientationError(recovered.orientation, regular_report.pose.orientation);
  printVector("\nIK solution", solution);
  std::cout << std::setprecision(6)
            << "FK/IK position error_m: " << position_error << "\n"
            << "FK/IK orientation error_rad: " << angle_error << "\n"
            << "manipulability regular / singular: "
            << regular_report.manipulability << " / " << singular_report.manipulability << "\n";

  const bool consistent = position_error < 1e-3 && angle_error < 1e-2;
  const bool singularity_visible = singular_report.manipulability < regular_report.manipulability;
  if (!consistent || !singularity_visible) {
    std::cerr << "Acceptance check failed\n";
    return finish(1);
  }

  std::cout << "Acceptance: FK, Jacobian, manipulability and FK/IK consistency passed\n";
  return finish(0);
}
