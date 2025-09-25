# CMakeLists.txt for comparison scripts

# Find required dependencies
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Set dependencies list for easier reuse
set(comparison_script_dependencies
  rclcpp
  tf2_ros
  tf2
  visualization_msgs
  geometry_msgs
  std_msgs
  nav_msgs
)

# Create executable for exploration_path_followed_tf
add_executable(exploration_path_followed_tf
  exploration_path_followed_tf.cpp
)
ament_target_dependencies(exploration_path_followed_tf ${comparison_script_dependencies})

# Create executable for explored_map_counter
add_executable(explored_map_counter
  explored_map_counter.cpp
)
ament_target_dependencies(explored_map_counter ${comparison_script_dependencies})

# Install the executables
install(TARGETS
  exploration_path_followed_tf
  explored_map_counter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
