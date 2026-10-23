cmake_minimum_required(VERSION 3.8)
project(robot_trajectory_optimization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(trajectory_core
  src/quintic_trajectory.cpp
  src/minimum_jerk.cpp
  src/minco.cpp)
target_include_directories(trajectory_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(trajectory_core PUBLIC Eigen3::Eigen)

add_executable(compare_trajectories src/compare_demo.cpp)
target_link_libraries(compare_trajectories trajectory_core)

install(TARGETS trajectory_core compare_trajectories
  EXPORT export_trajectory_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

ament_export_targets(export_trajectory_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectories test/test_trajectories.cpp)
  target_link_libraries(test_trajectories trajectory_core)
endif()

ament_package()
