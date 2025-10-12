# Information Gain Tests CMakeLists.txt

find_package(ament_cmake_gtest REQUIRED)

# Test dependencies
set(dependencies
  ament_cmake_gtest
  rclcpp
  nav2_util
  nav2_costmap_2d
  tf2_ros
  pluginlib
  roadmap_explorer_msgs
  geometry_msgs
  nav_msgs
  yaml-cpp
  Boost
  Eigen3
)

# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Test for BaseInformationGain
ament_add_gtest(test_base_information_gain
  test_base_information_gain.cpp
)
ament_target_dependencies(test_base_information_gain
  ${dependencies}
)
target_link_libraries(test_base_information_gain
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_information_gain_plugins
  roadmap_explorer_bt
)

# Test for CountBasedGain
ament_add_gtest(test_count_based_gain
  test_count_based_gain.cpp
)
ament_target_dependencies(test_count_based_gain
  ${dependencies}
)
target_link_libraries(test_count_based_gain
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_information_gain_plugins
  roadmap_explorer_bt
)
