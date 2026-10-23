cmake_minimum_required(VERSION 3.8)
project(robot_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_library(arm_kinematics src/arm_kinematics.cpp)
target_include_directories(arm_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(arm_kinematics Eigen3::Eigen)
ament_target_dependencies(arm_kinematics
  ament_index_cpp rclcpp moveit_core moveit_ros_planning)

add_executable(kinematics_demo src/kinematics_demo.cpp)
target_link_libraries(kinematics_demo arm_kinematics)
ament_target_dependencies(kinematics_demo rclcpp)

install(TARGETS arm_kinematics kinematics_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_include_directories(test_kinematics PUBLIC include)
  target_link_libraries(test_kinematics arm_kinematics)
  ament_target_dependencies(test_kinematics rclcpp)
endif()

ament_package()
