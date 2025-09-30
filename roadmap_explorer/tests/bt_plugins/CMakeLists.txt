# BT Plugins Tests CMakeLists.txt

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
  nav2_behavior_tree
)


# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


# Test for BaseBTPlugin
ament_add_gtest(test_base_bt_plugin
  test_base_bt_plugin.cpp
)
ament_target_dependencies(test_base_bt_plugin
  ${dependencies}
)
target_link_libraries(test_base_bt_plugin
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_bt_plugins
)


# Test for LogIterationPlugin
ament_add_gtest(test_log_iteration_plugin
  test_log_iteration_plugin.cpp
)
ament_target_dependencies(test_log_iteration_plugin
  ${dependencies}
)
target_link_libraries(test_log_iteration_plugin
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_bt_plugins
)


# Integration test for BT plugins
ament_add_gtest(test_bt_plugin_integration
  test_bt_plugin_integration.cpp
)
ament_target_dependencies(test_bt_plugin_integration
  ${dependencies}
)
target_link_libraries(test_bt_plugin_integration
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_bt_plugins
)
