cmake_minimum_required(VERSION 3.5)

# Find GTest
find_package(ament_cmake_gtest REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  nav2_util
  Eigen3
  nav2_costmap_2d
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  PCL
  pluginlib
)

# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Test executable for CostAssigner
ament_add_gtest(test_cost_assigner test_cost_assigner.cpp)
ament_target_dependencies(test_cost_assigner 
  ${dependencies}
)
target_link_libraries(test_cost_assigner
  roadmap_explorer_core
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_bt_plugins
  roadmap_explorer_frontier_search_plugins
  roadmap_explorer_planner_plugins
  roadmap_explorer_information_gain_plugins
)

# Set C++ standard for all tests
set_target_properties(test_cost_assigner PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)

