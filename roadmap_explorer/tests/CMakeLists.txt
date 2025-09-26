cmake_minimum_required(VERSION 3.5)

# Find GTest
find_package(ament_cmake_gtest REQUIRED)

# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Test executable for GeometryUtils
ament_add_gtest(test_geometry_utils test_geometry_utils.cpp)
ament_target_dependencies(test_geometry_utils 
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  nav2_util
  Eigen3
)

# Test executable for GeneralUtils
ament_add_gtest(test_general_utils test_general_utils.cpp)
ament_target_dependencies(test_general_utils 
  rclcpp
)

# Test executable for EventLogger
ament_add_gtest(test_event_logger test_event_logger.cpp 
  ${CMAKE_SOURCE_DIR}/src/util/EventLogger.cpp
)
ament_target_dependencies(test_event_logger 
  rclcpp
  nav2_util
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
