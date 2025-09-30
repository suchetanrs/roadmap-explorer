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
)

# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Test executable for GeometryUtils
ament_add_gtest(test_base_frontier_search test_base_frontier_search.cpp)
ament_target_dependencies(test_base_frontier_search 
  ${dependencies}
)
target_link_libraries(test_base_frontier_search
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_bt_plugins
  roadmap_explorer_frontier_search_plugins
)

# Test executable for GeneralUtils
ament_add_gtest(test_frontier_bf_search test_frontier_bf_search.cpp)
ament_target_dependencies(test_frontier_bf_search 
  ${dependencies}
)
target_link_libraries(test_frontier_bf_search
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_bt_plugins
  roadmap_explorer_frontier_search_plugins
)

# Set C++ standard for all tests
set_target_properties(test_base_frontier_search PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)

set_target_properties(test_frontier_bf_search PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)