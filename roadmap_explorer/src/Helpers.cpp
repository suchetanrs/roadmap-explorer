// helpers.cpp

#include "roadmap_explorer/Helpers.hpp" // Include the corresponding header file

namespace roadmap_explorer
{
void bresenham2D(
  RayTracedCells & at, unsigned int abs_da, unsigned int abs_db, int error_b,
  int offset_a,
  int offset_b, unsigned int offset,
  unsigned int max_length,
  int resolution_cut_factor,
  nav2_costmap_2d::Costmap2D * exploration_costmap_)
{
  auto max_offset = exploration_costmap_->getSizeInCellsX() *
    exploration_costmap_->getSizeInCellsY();
  unsigned int end = std::min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i) {
    if (i % resolution_cut_factor == 0) {
      at(offset);
    }
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da) {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  at(offset);
}

bool getTracedCells(
  double sx, double sy, double wx, double wy, RayTracedCells & cell_gatherer, double max_length,
  nav2_costmap_2d::Costmap2D * exploration_costmap_)
{
  unsigned int min_length = 0.0;
  int resolution_cut_factor = 1;
  // Calculate map coordinates
  unsigned int x1, y1;
  unsigned int x0, y0;
  if (!exploration_costmap_->worldToMap(
      wx, wy, x1,
      y1) || !exploration_costmap_->worldToMap(sx, sy, x0, y0))
  {
    std::cerr << "Not world to map" << std::endl;
    return false;
  }

  // Calculate distance and adjust starting point to min_length distance
  int dx_full = x1 - x0;
  int dy_full = y1 - y0;
  double dist = std::hypot(dx_full, dy_full);
  if (dist < min_length) {
    std::cerr <<
      "Distance to ray trace is lesser than minimum distance. Proceeding to next frontier." <<
      std::endl;
    return false;
  }
  unsigned int min_x0, min_y0;
  if (dist > 0.0) {
    // Adjust starting point and offset to start from min_length distance
    min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
    min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
  } else {
    min_x0 = x0;
    min_y0 = y0;
  }
  unsigned int offset = min_y0 * exploration_costmap_->getSizeInCellsX() + min_x0;

  int dx = x1 - min_x0;
  int dy = y1 - min_y0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * exploration_costmap_->getSizeInCellsX();

  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
  // Calculate the maximum number of steps based on resolution_cut_factor
  // if x is dominant
  if (abs_dx >= abs_dy) {
    int error_y = abs_dx / 2;

    roadmap_explorer::bresenham2D(
      cell_gatherer, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
      (unsigned int)(scale * abs_dx), resolution_cut_factor, exploration_costmap_);
  } else {
    // otherwise y is dominant
    int error_x = abs_dy / 2;
    roadmap_explorer::bresenham2D(
      cell_gatherer, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
      (unsigned int)(scale * abs_dy), resolution_cut_factor, exploration_costmap_);
  }
  return true;
}

bool surroundingCellsMapped(
  geometry_msgs::msg::Point & checkPoint,
  nav2_costmap_2d::Costmap2D & exploration_costmap_)
{
  unsigned int mx, my;
  if (!exploration_costmap_.worldToMap(checkPoint.x, checkPoint.y, mx, my)) {
    return false;
  }
  // return if the frontier is on a lethal cell.
  if (exploration_costmap_.getCost(mx, my) == 254) {
    return true;
  }
  auto out = nhood20(exploration_costmap_.getIndex(mx, my), exploration_costmap_);
  auto out8 = nhood8(exploration_costmap_.getIndex(mx, my), exploration_costmap_);
  int surrounding_lethal = 0;
  for (auto cell : out8) {
    auto cost = exploration_costmap_.getCost(cell);
    if (static_cast<int>(cost) == 254) {
      ++surrounding_lethal;
    }
  }
  if (surrounding_lethal >= 3) {
    return true;
  }
  for (auto cell : out) {
    auto cost = exploration_costmap_.getCost(cell);
    // // std::cout << "Cost is: " << static_cast<int>(cost) << std::endl;
    if (static_cast<int>(cost) == 255) {
      // // std::cout << "Surrounding cells mapped Returning false " << std::endl;
      return false;
    }
  }
  // // std::cout << "Surrounding cells mapped Returning true " << std::endl;
  return true;
}

bool isCircleFootprintInLethal(
  const nav2_costmap_2d::Costmap2D * costmap, unsigned int center_x,
  unsigned int center_y, double radius_in_cells)
{
  for (int dx = -radius_in_cells; dx <= radius_in_cells; ++dx) {
    for (int dy = -radius_in_cells; dy <= radius_in_cells; ++dy) {
      // Check if the point is within the circle
      if (dx * dx + dy * dy <= radius_in_cells * radius_in_cells) {
        unsigned int x = center_x + dx;
        unsigned int y = center_y + dy;
        unsigned int cost = costmap->getCost(x, y);
        if (cost == 254) {
          return true;               // Robot does not fit
        }
      }
    }
  }
  return false;       // Robot fits
}

// -------------------------- COSTMAP TOOLS ---------------------------------------------------------

/**
     * @brief Determine 4-connected neighbourhood of an input cell, checking for map edges
     * @param idx input cell index
     * @param costmap Reference to map data
     * @return neighbour cell indexes
     */
std::vector<unsigned int> nhood4(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap)
{
  // get 4-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out;

  unsigned int size_x_ = costmap.getSizeInCellsX();
  unsigned int size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    // std::cout << "Evaluating nhood for offmap point" << std::endl;
    return out;
  }

  if (idx % size_x_ > 0) {
    out.push_back(idx - 1);
  }
  if (idx % size_x_ < size_x_ - 1) {
    out.push_back(idx + 1);
  }
  if (idx >= size_x_) {
    out.push_back(idx - size_x_);
  }
  if (idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + size_x_);
  }
  return out;
}

/**
     * @brief Determine 8-connected neighbourhood of an input cell, checking for map edges
     * @param idx input cell index
     * @param costmap Reference to map data
     * @return neighbour cell indexes
     */
std::vector<unsigned int> nhood8(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap)
{
  // get 8-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out = nhood4(idx, costmap);

  unsigned int size_x_ = costmap.getSizeInCellsX();
  unsigned int size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    return out;
  }

  if (idx % size_x_ > 0 && idx >= size_x_) {
    out.push_back(idx - 1 - size_x_);
  }
  if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx - 1 + size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
    out.push_back(idx + 1 - size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + 1 + size_x_);
  }

  return out;
}

/**
     * @brief Determine 20-connected neighbourhood of an input cell, checking for map edges
     * @param idx input cell index
     * @param costmap Reference to map data
     * @return neighbour cell indexes
     */
std::vector<unsigned int> nhood20(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap)
{
  // get 8-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out = nhood4(idx, costmap);
  auto out_copy = out;
  for (auto val : out_copy) {
    auto out_4 = nhood8(val, costmap);
    out.insert(out.end(), out_4.begin(), out_4.end());
  }

  return out;
}

/**
     * @brief Find nearest cell of a specified value
     * @param result Index of located cell
     * @param start Index initial cell to search from
     * @param val free space threshold
     * @param costmap Reference to map data
     * @return True if a cell with the requested value was found
     */
bool nearestFreeCell(
  unsigned int & result, unsigned int start, unsigned char val,
  const nav2_costmap_2d::Costmap2D & costmap)
{

  const unsigned char * map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  if (start >= size_x * size_y) {
    return false;
  }

  // initialize breadth first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  // push initial cell
  bfs.push(start);
  visited_flag[start] = true;

  // search for neighbouring cell matching value
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // return if cell of correct value is found
    if (map[idx] < val) {
      result = idx;
      return true;
    }

    // iterate over all adjacent unvisited cells
    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
      }
    }
  }

  return false;
}

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

  // Run the A* search to compute the navigation function.
  if (!planner->calcNavFnDijkstra()) {
    LOG_WARN("Planner failed to compute the navigation function.");
    return false;
  }

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

bool computePathBetweenPointsThetaStar(
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
  auto planner = std::make_unique<roadmap_explorer::ThetaStar>();
  planner->costmap_ = exploration_costmap_;
  planner->how_many_corners_ = 8;
  planner->allow_unknown_ = planner_allow_unknown;
  planner->w_euc_cost_ = 1.0;
  planner->w_traversal_cost_ = 2.0;
  planner->w_heuristic_cost_ = planner->w_euc_cost_ < 1.0 ? planner->w_euc_cost_ : 1.0;
  planner->setStartAndGoal(start_point, goal_point);
  std::vector<coordsW> thetapath;
  if (planner->isUnsafeToPlan()) {
    LOG_ERROR("Either of the start or goal pose are an obstacle! ");
    return false;
  } else if (planner->generatePath(thetapath)) {
    planner->linearInterpolation(thetapath, exploration_costmap_->getResolution(), path);
    // global_path = linearInterpolation(thetapath, planner->costmap_->getResolution());
  } else {
    LOG_ERROR("Could not generate path between the given poses");
    return false;
  }
  return true;
}

Eigen::Affine3f getTransformFromPose(geometry_msgs::msg::Pose & pose)
{
  // Extract translation and rotation from the pose message
  Eigen::Vector3f translation(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaternionf rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);

  // Construct the transformation matrix
  Eigen::Affine3f transform = Eigen::Translation3f(translation) * Eigen::Quaternionf(rotation);

  return transform;
}
}
