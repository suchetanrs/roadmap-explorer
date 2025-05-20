cmake_minimum_required(VERSION 3.5)
project(roadmap_explorer)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(roadmap_explorer_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_util)
find_package(PCL REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(dependencies
ament_cmake
rclcpp
rclcpp_action
geometry_msgs
action_msgs
nav2_msgs
rosidl_default_generators
nav2_costmap_2d
pcl_ros
sensor_msgs
tf2_ros
tf2
pluginlib
roadmap_explorer_msgs
visualization_msgs
eigen3_cmake_module
Eigen3
PCL
nav2_util
example_interfaces
behaviortree_cpp
yaml-cpp
nav2_navfn_planner
)

# Add compile definitions for visualization flags
add_compile_definitions(
  FRONTIER_POINT_MEDIAN
  # FRONTIER_POINT_INITIAL
)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Add executable 2
add_executable(explore_server 
src/controllers/recovery_controller.cpp 
src/controllers/initialization_controller.cpp 
src/planners/theta_star.cpp
src/planners/FrontierRoadmap.cpp
src/planners/astar.cpp
src/util/EventLogger.cpp
src/util/RosVisualizer.cpp
src/Parameters.cpp
src/Frontier.cpp 
src/Nav2Interface.cpp 
src/Helpers.cpp
src/FrontierSearch.cpp
# # src/FrontierSearchAllCells.cpp
src/CostCalculator.cpp
src/main.cpp
src/ExplorationBT.cpp 
src/CostAssigner.cpp
src/FullPathOptimizer.cpp 
)
ament_target_dependencies(explore_server ${dependencies})
target_link_libraries(explore_server ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})
# ------------------------------------------------------------------


install(TARGETS explore_server
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params include xml
DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
pluginlib_export_plugin_description_file(nav2_core xml/roadmap_planner.xml)
ament_package()
