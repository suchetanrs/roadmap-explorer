#include "roadmap_explorer/CostCalculator.hpp"

namespace roadmap_explorer
{
FrontierCostCalculator::FrontierCostCalculator(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
{
  LOG_INFO("FrontierCostCalculator::FrontierCostCalculator");
  exploration_costmap_ = explore_costmap_ros->getCostmap();
  // RosVisualizerInstance = std::make_shared<RosVisualizer>(node, exploration_costmap_);
  robot_radius_ = explore_costmap_ros->getRobotRadius();
}

FrontierCostCalculator::~FrontierCostCalculator()
{
  LOG_INFO("FrontierCostCalculator::~FrontierCostCalculator()");
}

void FrontierCostCalculator::setArrivalInformationForFrontier(
  FrontierPtr & frontier,
  std::vector<double> & polygon_xy_min_max)
{
  double sx, sy;       // sensor x, sensor y, sensor orientation
  double wx, wy;
  unsigned int max_length = MAX_CAMERA_DEPTH / (exploration_costmap_->getResolution());
  sx = frontier->getGoalPoint().x;
  sy = frontier->getGoalPoint().y;
  std::vector<int> information_along_ray;       // stores the information along each ray in 2PI.
  std::vector<geometry_msgs::msg::Pose> vizpoints;
  float hitObstacleCount = 0;
  for (double theta = 0; theta <= (2 * M_PI); theta += DELTA_THETA) {
    std::vector<nav2_costmap_2d::MapLocation> traced_cells;
    // treats cells 240 to 254 as obstacles and returns 255 in the traced cells.
    RayTracedCells cell_gatherer(exploration_costmap_, traced_cells, 240, 254, 255, 255);

    wx = sx + (MAX_CAMERA_DEPTH * cos(theta));
    wy = sy + (MAX_CAMERA_DEPTH * sin(theta));

    // Check if wx and wy are outside the polygon. If they are, bring it to the edge of the polygon.
    // This is to prevent raytracing beyond the edge of the boundary polygon.
    wx =
      std::max(
      polygon_xy_min_max[0],
      std::max(
        exploration_costmap_->getOriginX(),
        std::min(
          polygon_xy_min_max[2],
          std::min(
            exploration_costmap_->getOriginX() + exploration_costmap_->getSizeInMetersX(), wx))));
    wy =
      std::max(
      polygon_xy_min_max[1],
      std::max(
        exploration_costmap_->getOriginY(),
        std::min(
          polygon_xy_min_max[3],
          std::min(
            exploration_costmap_->getOriginY() + exploration_costmap_->getSizeInMetersY(), wy))));

    if (!getTracedCells(sx, sy, wx, wy, cell_gatherer, max_length, exploration_costmap_)) {
      frontier->setArrivalInformation(0.0);
      frontier->setGoalOrientation(0.0);
      return;
    }

    auto info_addition = cell_gatherer.getCells();
    information_along_ray.push_back(info_addition.size());
    if (cell_gatherer.hasHitObstacle()) {
      ++hitObstacleCount;
    }
    // loop for visualization
    for (size_t counter_info = 0; counter_info < info_addition.size(); counter_info++) {
      double wmx, wmy;
      exploration_costmap_->mapToWorld(
        info_addition[counter_info].x, info_addition[counter_info].y,
        wmx, wmy);
      geometry_msgs::msg::Pose pnts;
      pnts.position.x = wmx;
      pnts.position.y = wmy;
      vizpoints.push_back(pnts);
    }
  } // theta end

  unsigned int sxm, sym;
  if (!exploration_costmap_->worldToMap(sx, sy, sxm, sym)) {
    LOG_ERROR("The detected frontier is outside the map. What is going on?")
    throw std::runtime_error("The detected frontier is outside the map. What is going on?");
  }
  bool footprintInLethalPenalty = isCircleFootprintInLethal(
    exploration_costmap_, sxm, sym, std::ceil(
      robot_radius_ / exploration_costmap_->getResolution()));
  // TODO(suchetan): Parametrize this value that compares against getSize()
  if (footprintInLethalPenalty && frontier->getSize() < 10.0) {
    LOG_DEBUG("Frontier " << frontier << " is not achievable. Very close to lethal obstacle.");
    frontier->setAchievability(false);
  }

  std::vector<int> kernel(static_cast<int>(CAMERA_FOV / DELTA_THETA), 1);       // initialize a kernal vector of size 6 and all elements = 1
  int n = information_along_ray.size();                                         // number of rays computed in 2PI
  int k = kernel.size();
  std::vector<int> result(n - k + 1, 0);
  for (int i = 0; i < n - k + 1; ++i) {
    for (int j = 0; j < k; ++j) {
      result[i] += information_along_ray[i + j] * kernel[j];
    }
  }
  int maxIndex = 0;
  int maxValue = result[0];
  for (int i = 1; i < (int)result.size(); ++i) {
    if (result[i] > maxValue) {
      maxValue = result[i];
      maxIndex = i;
    }
  }
  LOG_DEBUG(
    "Total unknown cells is: " +
    std::to_string(
      std::accumulate(
        information_along_ray.begin(), information_along_ray.end(),
        0)));

  // visualize raytraced points
  // RosVisualizer::getInstance()observableCellsViz(vizpoints);
  frontier->setArrivalInformation(maxValue);
  LOG_DEBUG("Arrival information is: " << frontier->getArrivalInformation());
  if (frontier->getArrivalInformation() < min_arrival_info_gt_) {
    LOG_DEBUG("FrontierPtr " << *frontier << " is not achievable. Arrival information is too low.");
    frontier->setAchievability(false);
  }
  frontier->setGoalOrientation((maxIndex * DELTA_THETA) + (CAMERA_FOV / 2));
  return;
}

double FrontierCostCalculator::setArrivalInformationLimits()
{
  // LOG_WARN("Setting arrival information limits.");
  if (arrival_info_limits_set_) {
    // LOG_WARN("Arrival information limits already set.");
    return 0.0;
  }
  double sx, sy;       // sensor x, sensor y, sensor orientation
  double wx, wy;
  unsigned int max_length = MAX_CAMERA_DEPTH / (exploration_costmap_->getResolution());
  sx = exploration_costmap_->getOriginX() + (exploration_costmap_->getSizeInMetersX() / 2);
  sy = exploration_costmap_->getOriginY() + (exploration_costmap_->getSizeInMetersY() / 2);
  std::vector<int> information_along_ray;       // stores the information along each ray in 2PI.
  std::vector<geometry_msgs::msg::Pose> vizpoints;
  for (double theta = 0; theta <= (2 * M_PI); theta += DELTA_THETA) {
    std::vector<nav2_costmap_2d::MapLocation> traced_cells;
    // treats cells 240 to 254 as obstacles and returns 255 in the traced cells.
    RayTracedCells cell_gatherer(exploration_costmap_, traced_cells, 260, 260, 0, 255);

    wx = sx + (MAX_CAMERA_DEPTH * cos(theta));
    wy = sy + (MAX_CAMERA_DEPTH * sin(theta));

    if (!getTracedCells(sx, sy, wx, wy, cell_gatherer, max_length, exploration_costmap_)) {
      LOG_ERROR("Error in raytracing. Cannot set arrival information limits.");
      LOG_ERROR("Max length is: " << max_length);
      throw std::runtime_error(
              "Error in raytracing. Cannot set arrival information limits.");
      return 0;
    }

    auto info_addition = cell_gatherer.getCells();
    information_along_ray.push_back(info_addition.size());
    // loop for visualization
    for (size_t counter_info = 0; counter_info < info_addition.size(); counter_info++) {
      double wmx, wmy;
      exploration_costmap_->mapToWorld(
        info_addition[counter_info].x, info_addition[counter_info].y,
        wmx, wmy);
      geometry_msgs::msg::Pose pnts;
      pnts.position.x = wmx;
      pnts.position.y = wmy;
      vizpoints.push_back(pnts);
    }
  }       // theta end

  std::vector<int> kernel(static_cast<int>(CAMERA_FOV / DELTA_THETA), 1);       // initialize a kernal vector of size 6 and all elements = 1
  int n = information_along_ray.size();                                         // number of rays computed in 2PI
  int k = kernel.size();
  std::vector<int> result(n - k + 1, 0);
  for (int i = 0; i < n - k + 1; ++i) {
    for (int j = 0; j < k; ++j) {
      result[i] += information_along_ray[i + j] * kernel[j];
    }
  }
  int maxValue = result[0];
  for (int i = 1; i < (int)result.size(); ++i) {
    if (result[i] > maxValue) {
      maxValue = result[i];
    }
  }
  arrival_info_limits_set_ = true;
  max_arrival_info_gt_ = maxValue * 1.2;
  LOG_WARN("Max arrival cost GT: " << max_arrival_info_gt_);
  min_arrival_info_gt_ = factor_of_max_is_min * max_arrival_info_gt_;
  LOG_WARN("Min arrival cost GT: " << min_arrival_info_gt_);
  return maxValue;
}

void FrontierCostCalculator::setPlanForFrontier(
  geometry_msgs::msg::Pose start_pose_w,
  FrontierPtr & goal_point_w)
{
  if (goal_point_w->isAchievable() == false) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }
  auto start_point_w = start_pose_w.position;

  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  std::unique_ptr<nav2_navfn_planner::NavFn> planner_;
  planner_ = std::make_unique<nav2_navfn_planner::NavFn>(
    exploration_costmap_->getSizeInCellsX(), exploration_costmap_->getSizeInCellsY());
  planner_->setNavArr(
    exploration_costmap_->getSizeInCellsX(),
    exploration_costmap_->getSizeInCellsY());
  planner_->setCostmap(exploration_costmap_->getCharMap(), true, planner_allow_unknown_);

  // start point
  unsigned int mx, my;
  if (!exploration_costmap_->worldToMap(start_point_w.x, start_point_w.y, mx, my)) {
    LOG_ERROR(
      "Cannot create a plan: the robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }
  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  // goal point
  if (!exploration_costmap_->worldToMap(
      goal_point_w->getGoalPoint().x,
      goal_point_w->getGoalPoint().y, mx, my))
  {
    LOG_ERROR(
      "The goal sent to the planner is off the global costmap Planning will always fail to this goal.");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // Take note this is computed backwards. Copied what was done in nav2 navfn planner.
  planner_->setStart(map_goal);
  planner_->setGoal(map_start);

  if (!planner_->calcNavFnAstar()) {
    LOG_WARN(
      "Plan not Found for frontier at x: " << goal_point_w->getGoalPoint().x << " y: " <<
        goal_point_w->getGoalPoint().y);
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  const int & max_cycles =
    (exploration_costmap_->getSizeInCellsX() >=
    exploration_costmap_->getSizeInCellsY()) ? (exploration_costmap_->getSizeInCellsX() *
    4) : (exploration_costmap_->getSizeInCellsY() * 4);
  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    LOG_INFO("Plan not found for " << goal_point_w << " path length is zero.");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();
  double path_length_m = 0;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> previous_pose = nullptr;
  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    exploration_costmap_->mapToWorld(x[i], y[i], world_x, world_y);
    geometry_msgs::msg::PoseStamped pose_from;
    pose_from.pose.position.x = world_x;
    pose_from.pose.position.y = world_y;
    pose_from.pose.position.z = 0.0;
    plan.poses.push_back(pose_from);
    if (i != 0 && previous_pose != nullptr) {
      path_length_m += distanceBetweenPoints(pose_from.pose.position, previous_pose->pose.position);
    }
    previous_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(pose_from);
  }
  if (plan.poses.size() == 0) {
    LOG_INFO("Plan not found for " << goal_point_w << " path length is zero.");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  if (path_length_m < closeness_rejection_threshold_) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  goal_point_w->setAchievability(true);
  goal_point_w->setPathLength(plan.poses.size());
  goal_point_w->setPathLengthInM(path_length_m);
  goal_point_w->setPathHeading(std::numeric_limits<double>::max());
  return;
}

void FrontierCostCalculator::setPlanForFrontierRoadmap(
  geometry_msgs::msg::Pose start_pose_w,
  FrontierPtr & goal_point_w)
{
  PROFILE_FUNCTION;
  if (goal_point_w->isAchievable() == false) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }
  auto path_length = FrontierRoadMap::getInstance().getPlan(
    start_pose_w.position.x,
    start_pose_w.position.y, true,
    goal_point_w->getGoalPoint().x, goal_point_w->getGoalPoint().y, true, false);
  if (path_length.path_exists == false) {
    LOG_INFO("Plan not found for " << goal_point_w << " path does not exist.");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  if (path_length.path_length_m < closeness_rejection_threshold_) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  goal_point_w->setAchievability(true);
  goal_point_w->setPathLength(path_length.path_length_m);
  goal_point_w->setPathLengthInM(path_length.path_length_m);
  goal_point_w->setPathHeading(std::numeric_limits<double>::max());
  return;
}

void FrontierCostCalculator::setPlanForFrontierEuclidean(
  geometry_msgs::msg::Pose start_pose_w,
  FrontierPtr & goal_point_w)
{
  auto start_point_w = start_pose_w.position;
  if (goal_point_w->isAchievable() == false) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }
  auto length_to_frontier =
    sqrt(
    pow(
      start_point_w.x - goal_point_w->getGoalPoint().x,
      2) + pow(start_point_w.y - goal_point_w->getGoalPoint().y, 2));
  if (length_to_frontier < closeness_rejection_threshold_) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }
  goal_point_w->setAchievability(true);
  goal_point_w->setPathLength(length_to_frontier);
  goal_point_w->setPathLengthInM(length_to_frontier);
  goal_point_w->setPathHeading(std::numeric_limits<double>::max());
  return;
}

double FrontierCostCalculator::getRandomVal()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  return dis(gen);
}

void FrontierCostCalculator::setRandomMetaData(FrontierPtr & goal_point_w)
{
  goal_point_w->setArrivalInformation(getRandomVal());
  goal_point_w->setGoalOrientation(getRandomVal());
  goal_point_w->setPathLength(getRandomVal());
}

void FrontierCostCalculator::setClosestFrontierMetaData(
  geometry_msgs::msg::Pose start_pose_w,
  FrontierPtr & goal_point_w)
{
  goal_point_w->setArrivalInformation(1e-9);
  goal_point_w->setGoalOrientation(1e-9);
  setPlanForFrontierEuclidean(start_pose_w, goal_point_w);
}

void FrontierCostCalculator::recomputeNormalizationFactors(FrontierPtr & frontier)
{
  if (!frontier->isAchievable()) {
    return;
  }
  min_traversable_distance = std::min(min_traversable_distance, frontier->getPathLength());
  max_traversable_distance = std::max(max_traversable_distance, frontier->getPathLength());
  min_arrival_info_per_frontier = std::min(
    min_arrival_info_per_frontier,
    frontier->getArrivalInformation());
  max_arrival_info_per_frontier = std::max(
    max_arrival_info_per_frontier,
    frontier->getArrivalInformation());
}
}
