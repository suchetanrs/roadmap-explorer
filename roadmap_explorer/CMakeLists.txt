cmake_minimum_required(VERSION 3.5)
project(roadmap_explorer)

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
find_package(yaml-cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(Boost REQUIRED)

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
  nav2_behavior_tree
)

####################### GET ROS DISTRO ##########################

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO environment variable not set—are you sourcing your ROS2 installation?")
endif()

string(TOUPPER "$ENV{ROS_DISTRO}" ROS_DISTRO_UPPER)
message(STATUS "Building for ROS distro: ${ROS_DISTRO_UPPER}")
add_compile_definitions(ROS_DISTRO_${ROS_DISTRO_UPPER})

# add_compile_definitions(
#   USE_RCLCPP_LOGGER
# )


######################## INCLUDE DIRECTORIES #########################

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

########################### Libraries ###########################
add_library(${PROJECT_NAME}_frontier SHARED
src/Frontier.cpp)
ament_target_dependencies(${PROJECT_NAME}_frontier ${dependencies})


add_library(${PROJECT_NAME}_utils SHARED
src/util/EventLogger.cpp
src/util/RosVisualizer.cpp
src/Helpers.cpp
)
ament_target_dependencies(${PROJECT_NAME}_utils ${dependencies})
target_link_libraries(${PROJECT_NAME}_utils ${PROJECT_NAME}_frontier)


add_library(${PROJECT_NAME}_frontier_search_plugins SHARED 
src/frontier_search/PluginBFSearch.cpp
)
ament_target_dependencies(${PROJECT_NAME}_frontier_search_plugins ${dependencies})


add_library(${PROJECT_NAME}_information_gain_plugins SHARED
src/information_gain/CountBasedGain.cpp
)
ament_target_dependencies(${PROJECT_NAME}_information_gain_plugins ${dependencies})


add_library(${PROJECT_NAME}_planner_plugins SHARED
src/planners/ThetaStar.cpp
src/planners/Astar.cpp
src/planners/NavFn.cpp
src/planners/NavFnPlugin.cpp
src/planners/FrontierRoadmap.cpp
src/planners/FrontierRoadmapPlugin.cpp
src/planners/EuclideanPlugin.cpp
src/planners/RandomDistancePlugin.cpp
)
ament_target_dependencies(${PROJECT_NAME}_planner_plugins ${dependencies})


add_library(${PROJECT_NAME}_core SHARED
src/CostAssigner.cpp
src/Parameters.cpp
src/Nav2Interface.cpp 
src/FullPathOptimizer.cpp 
src/SensorSimulator.cpp
)
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})
target_link_libraries(${PROJECT_NAME}_core 
${PCL_LIBRARIES} 
${PROJECT_NAME}_frontier 
${PROJECT_NAME}_utils 
${PROJECT_NAME}_frontier_search_plugins 
${PROJECT_NAME}_planner_plugins 
${PROJECT_NAME}_information_gain_plugins
)


add_library(${PROJECT_NAME}_bt_plugins SHARED
src/bt_plugins/LogIterationPlugin.cpp
src/bt_plugins/UpdateBoundaryPlugin.cpp
src/bt_plugins/SearchForFrontiersPlugin.cpp
src/bt_plugins/CleanupRoadmapPlugin.cpp
src/bt_plugins/UpdateRoadmapPlugin.cpp
src/bt_plugins/ProcessFrontierCostsPlugin.cpp
src/bt_plugins/OptimizeFullPathPlugin.cpp
src/bt_plugins/SendNav2GoalPlugin.cpp
src/bt_plugins/BlacklistGoalPlugin.cpp
)
ament_target_dependencies(${PROJECT_NAME}_bt_plugins ${dependencies})
target_link_libraries(${PROJECT_NAME}_bt_plugins 
${PROJECT_NAME}_frontier 
${PROJECT_NAME}_utils
${PROJECT_NAME}_frontier_search_plugins 
${PROJECT_NAME}_planner_plugins 
${PROJECT_NAME}_information_gain_plugins
)


add_library(${PROJECT_NAME}_bt SHARED
src/ExplorationBT.cpp
src/ExplorationServer.cpp
)
ament_target_dependencies(${PROJECT_NAME}_bt ${dependencies})
target_link_libraries(${PROJECT_NAME}_bt
${PROJECT_NAME}_bt_plugins 
${PROJECT_NAME}_frontier 
${PROJECT_NAME}_utils
${PROJECT_NAME}_frontier_search_plugins 
${PROJECT_NAME}_planner_plugins 
${PROJECT_NAME}_information_gain_plugins
${PROJECT_NAME}_core
)

rclcpp_components_register_nodes(${PROJECT_NAME}_bt "roadmap_exploration::ExplorationServer")

# ########################### EXECUTABLES ###########################
add_executable(roadmap_exploration_server
src/main.cpp
)
ament_target_dependencies(roadmap_exploration_server ${dependencies})
target_link_libraries(roadmap_exploration_server 
${PCL_LIBRARIES} 
${PROJECT_NAME}_frontier 
${PROJECT_NAME}_bt 
${PROJECT_NAME}_utils 
${PROJECT_NAME}_bt_plugins 
${PROJECT_NAME}_frontier_search_plugins 
${PROJECT_NAME}_planner_plugins 
${PROJECT_NAME}_information_gain_plugins
${PROJECT_NAME}_core
)

########################### COMPARISON SCRIPTS ###########################
add_subdirectory(comparision_scripts)

########################### INSTALLATION ###########################

install(TARGETS
  ${PROJECT_NAME}_frontier
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_frontier_search_plugins
  ${PROJECT_NAME}_information_gain_plugins
  ${PROJECT_NAME}_planner_plugins
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_bt_plugins
  ${PROJECT_NAME}_bt
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
ament_export_libraries(
${PROJECT_NAME}_frontier 
${PROJECT_NAME}_utils 
${PROJECT_NAME}_frontier_search_plugins 
${PROJECT_NAME}_information_gain_plugins
${PROJECT_NAME}_planner_plugins 
${PROJECT_NAME}_core
${PROJECT_NAME}_bt_plugins 
${PROJECT_NAME}_bt 
)
ament_export_dependencies(rosidl_default_runtime ${dependencies})

install(FILES core_plugins_description.xml frontier_search_plugins.xml frontier_planner_plugins.xml
        DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(roadmap_explorer core_plugins_description.xml)
pluginlib_export_plugin_description_file(roadmap_explorer frontier_search_plugins.xml)
pluginlib_export_plugin_description_file(roadmap_explorer frontier_planner_plugins.xml)
pluginlib_export_plugin_description_file(roadmap_explorer frontier_information_gain_plugins.xml)

########################### TESTS ###########################
find_package(ament_cmake_gtest QUIET)
if(ament_cmake_gtest_FOUND)
    add_subdirectory(tests/utils)
    # add_subdirectory(tests/bt_plugins)
    # add_subdirectory(tests/frontier_search)
    # add_subdirectory(tests/information_gain)
    # add_subdirectory(tests/other_tests)
endif()

ament_package()
