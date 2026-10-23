cmake_minimum_required(VERSION 3.8)
project(robot_trajectory_execution)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_kinematics REQUIRED)
find_package(robot_motion_planning REQUIRED)
find_package(robot_trajectory_optimization REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(trajectory_checks src/trajectory_checks.cpp)
target_include_directories(trajectory_checks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_checks
  geometry_msgs moveit_core moveit_msgs moveit_ros_planning rclcpp shape_msgs trajectory_msgs)
target_link_libraries(trajectory_checks robot_trajectory_optimization::trajectory_core)

add_executable(experiment_runner src/experiment_runner.cpp)
target_link_libraries(experiment_runner
  trajectory_checks
  robot_trajectory_optimization::trajectory_core
  robot_kinematics::arm_kinematics)
ament_target_dependencies(experiment_runner rclcpp moveit_ros_planning)

add_executable(execute_trajectory src/execute_trajectory.cpp)
target_link_libraries(execute_trajectory robot_trajectory_optimization::trajectory_core)
ament_target_dependencies(execute_trajectory control_msgs rclcpp rclcpp_action sensor_msgs trajectory_msgs)

add_executable(retime_moveit_path src/retime_moveit_path.cpp)
target_link_libraries(retime_moveit_path
  trajectory_checks
  robot_trajectory_optimization::trajectory_core
  robot_motion_planning::motion_planner)
ament_target_dependencies(retime_moveit_path
  rclcpp moveit_ros_planning moveit_ros_planning_interface trajectory_msgs)

install(TARGETS trajectory_checks experiment_runner execute_trajectory retime_moveit_path
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
