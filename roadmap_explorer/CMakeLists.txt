cmake_minimum_required(VERSION 3.5)
project(roadmap_explorer)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(roadmap_explorer_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

set(dependencies
ament_cmake
rclcpp
rclcpp_action
rosidl_default_generators
geometry_msgs
sensor_msgs
action_msgs
visualization_msgs
nav2_msgs
nav2_costmap_2d
nav2_navfn_planner
nav2_util
nav2_map_server
pcl_ros
PCL
tf2_ros
tf2
pluginlib
roadmap_explorer_msgs
eigen3_cmake_module
Eigen3
behaviortree_cpp_v3
yaml-cpp
nav2_behavior_tree
)

# add_compile_definitions(
#   USE_RCLCPP_LOGGER
# )

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

#-------------------------------------------------------------------
add_library(roadmap_exploration_bt SHARED
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
src/CostCalculator.cpp
src/ExplorationBT.cpp
src/CostAssigner.cpp
src/FullPathOptimizer.cpp 
src/ExplorationServer.cpp
src/SensorSimulator.cpp
)
ament_target_dependencies(roadmap_exploration_bt ${dependencies})
target_link_libraries(roadmap_exploration_bt ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})

rclcpp_components_register_nodes(roadmap_exploration_bt "roadmap_exploration::ExplorationServer")

#-------------------------------------------------------------------
add_executable(roadmap_exploration_server
src/main.cpp
)
ament_target_dependencies(roadmap_exploration_server ${dependencies})
target_link_libraries(roadmap_exploration_server ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} roadmap_exploration_bt)

# ------------------------------------------------------------------
# add_executable(sensor_simulator src/SensorSimulator.cpp)
# ament_target_dependencies(sensor_simulator ${dependencies})



install(TARGETS roadmap_exploration_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS roadmap_exploration_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch params include xml rviz
DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_export_libraries(roadmap_exploration_bt roadmap_exploration_server)
ament_export_dependencies(rosidl_default_runtime ${dependencies})
ament_package()
