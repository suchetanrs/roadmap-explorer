#include <roadmap_explorer/planners/NavFn.hpp>


namespace roadmap_explorer
{

bool computePathBetweenPoints(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & start_point,
  const geometry_msgs::msg::Point & goal_point,
  bool planner_allow_unknown, nav2_costmap_2d::Costmap2D * exploration_costmap_)
{
  unsigned int mx, my;

  // Convert start point from world to map coordinates.
  if (!exploration_costmap_->worldToMap(start_point.x, start_point.y, mx, my)) {
    LOG_ERROR("Start point is off the global costmap.");
    return false;
  }
  int map_start[2] = {static_cast<int>(mx), static_cast<int>(my)};

  // Convert goal point from world to map coordinates.
  if (!exploration_costmap_->worldToMap(goal_point.x, goal_point.y, mx, my)) {
    LOG_ERROR("Goal point is off the global costmap.");
    return false;
  }
  int map_goal[2] = {static_cast<int>(mx), static_cast<int>(my)};

  // Create and configure the planner.
  auto planner = std::make_unique<nav2_navfn_planner::NavFn>(
    exploration_costmap_->getSizeInCellsX(), exploration_costmap_->getSizeInCellsY());
  planner->setNavArr(
    exploration_costmap_->getSizeInCellsX(),
    exploration_costmap_->getSizeInCellsY());
  planner->setCostmap(exploration_costmap_->getCharMap(), true, planner_allow_unknown);

  // Note: The planner expects the start and goal in reverse order.
  planner->setStart(map_goal);
  planner->setGoal(map_start);

#ifdef ROS_DISTRO_HUMBLE
  // Run the A* search to compute the navigation function.
  if (!planner->calcNavFnDijkstra()) {
    LOG_WARN("Planner failed to compute the navigation function.");
    return false;
  }
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  // Run the A* search to compute the navigation function.
  std::function<bool()> cancelChecker = []() {
      return !rclcpp::ok();
    };
  if (!planner->calcNavFnDijkstra(cancelChecker)) {
    LOG_WARN("Planner failed to compute the navigation function.");
    return false;
  }
#else
  #error Unsupported ROS DISTRO
#endif

  // Determine the maximum cycles to search.
  const int max_cycles =
    (exploration_costmap_->getSizeInCellsX() >= exploration_costmap_->getSizeInCellsY()) ?
    (exploration_costmap_->getSizeInCellsX() * 4) :
    (exploration_costmap_->getSizeInCellsY() * 4);
  const int path_len = planner->calcPath(max_cycles);
  if (path_len == 0) {
    LOG_WARN("No path found between the given points.");
    return false;
  }

  // Extract the computed path.
  float * path_x = planner->getPathX();
  float * path_y = planner->getPathY();
  const int len = planner->getPathLen();

  // The path is generated in reverse order; iterate from end to beginning.
  for (int i = len - 1; i >= 0; --i) {
    double world_x, world_y;
    exploration_costmap_->mapToWorld(path_x[i], path_y[i], world_x, world_y);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    path.poses.push_back(pose);
  }
  return true;
}

}
