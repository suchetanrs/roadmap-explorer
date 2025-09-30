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
ament_add_gtest(test_geometry_utils test_geometry_utils.cpp)
ament_target_dependencies(test_geometry_utils 
  ${dependencies}
)

# Test executable for GeneralUtils
ament_add_gtest(test_general_utils test_general_utils.cpp)
ament_target_dependencies(test_general_utils 
  ${dependencies}
)

# Test executable for EventLogger
ament_add_gtest(test_event_logger test_event_logger.cpp 
  ${CMAKE_SOURCE_DIR}/src/util/EventLogger.cpp
)
ament_target_dependencies(test_event_logger 
  ${dependencies}
)

# Test executable for RosVisualizer
ament_add_gtest(test_ros_visualizer test_ros_visualizer.cpp 
  ${CMAKE_SOURCE_DIR}/src/util/RosVisualizer.cpp
  ${CMAKE_SOURCE_DIR}/src/Frontier.cpp
)
ament_target_dependencies(test_ros_visualizer 
  ${dependencies}
)
target_link_libraries(test_ros_visualizer ${PCL_LIBRARIES})

# Test executable for Helpers
ament_add_gtest(test_helpers test_helpers.cpp 
  ${CMAKE_SOURCE_DIR}/src/Helpers.cpp
)
ament_target_dependencies(test_helpers 
  ${dependencies}
)

# Set C++ standard for all tests
set_target_properties(test_geometry_utils PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)

set_target_properties(test_general_utils PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)

set_target_properties(test_event_logger PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)

set_target_properties(test_ros_visualizer PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)

set_target_properties(test_helpers PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)