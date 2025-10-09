#include "roadmap_explorer/information_gain/CountBasedGain.hpp"

namespace roadmap_explorer
{

void CountBasedGain::configure(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
{
  LOG_INFO("CountBasedGain::configure");
  exploration_costmap_ = explore_costmap_ros->getCostmap();
  explore_costmap_ros_ = explore_costmap_ros;
  arrival_info_limits_set_ = false;
  updateParameters();
  setArrivalInformationLimits();
}

void CountBasedGain::reset()
{
  arrival_info_limits_set_ = false;
  min_arrival_info_gt_ = std::numeric_limits<double>::max();
  max_arrival_info_gt_ = -1.0 * std::numeric_limits<double>::max();
}

void CountBasedGain::setInformationGainForFrontier(
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
      throw RoadmapExplorerException("The detected frontier is outside the map. What is going on?");
    }
    bool footprintInLethalPenalty = isCircleFootprintInLethal(
      exploration_costmap_, sxm, sym, std::ceil(
        explore_costmap_ros_->getRobotRadius() / exploration_costmap_->getResolution()));
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
    // rosVisualizerInstance.visualizeMarkers("observable_cells", vizPoints);
    frontier->setArrivalInformation(maxValue);
    LOG_DEBUG("Arrival information is: " << frontier->getArrivalInformation());
    if (frontier->getArrivalInformation() < min_arrival_info_gt_) {
      LOG_DEBUG("FrontierPtr " << *frontier << " is not achievable. Arrival information is too low.");
      frontier->setAchievability(false);
    }
    frontier->setGoalOrientation((maxIndex * DELTA_THETA) + (CAMERA_FOV / 2));
    return;
}

double CountBasedGain::setArrivalInformationLimits()
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
      throw RoadmapExplorerException(
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
} // namespace roadmap_explorer

#include <pluginlib/class_list_macros.hpp>
// Register the plugin
PLUGINLIB_EXPORT_CLASS(roadmap_explorer::CountBasedGain, roadmap_explorer::BaseInformationGain)