#ifndef COST_ASSIGNER_HPP_
#define COST_ASSIGNER_HPP_

#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/geometry_utils.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/RosVisualizer.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"
#include "roadmap_explorer/util/GeneralUtils.hpp"

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/information_gain/BaseInformationGain.hpp"
#include "roadmap_explorer/frontier_search/BaseFrontierSearch.hpp"
#include "roadmap_explorer/planners/BasePlanner.hpp"

namespace roadmap_explorer
{

struct CalculateFrontierCostsRequest
{
  geometry_msgs::msg::PoseStamped start_pose;
  std::vector<FrontierPtr> frontier_list;
  std::vector<std::vector<double>> every_frontier;
  std::vector<FrontierPtr> prohibited_frontiers;
};

class CostAssigner
{
public:
  CostAssigner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

  ~CostAssigner();

  void updateBoundaryPolygon(geometry_msgs::msg::PolygonStamped & explore_boundary);

  bool populateFrontierCosts(std::shared_ptr<CalculateFrontierCostsRequest> requestData);

private:

  bool assignCosts(
    std::vector<FrontierPtr> & frontier_list, geometry_msgs::msg::Pose start_pose_w);

  void setFrontierBlacklist(std::vector<FrontierPtr> & blacklist);

  void recomputeNormalizationFactors(FrontierPtr & frontier);

  std::vector<FrontierPtr> findDuplicates(const std::vector<FrontierPtr> & vec);

  void initializePlugins(const std::string & informationGainPlugin, const std::string & plannerPlugin);

  void processFrontier(FrontierPtr & frontier,
    const geometry_msgs::msg::Pose & start_pose_w);

  void setFrontierUnachievable(FrontierPtr & frontier);
  
  // min_x, min_y, max_x, max_y
  std::vector<double> polygon_xy_min_max_;

  nav2_costmap_2d::LayeredCostmap * layered_costmap_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;

  // map of blacklists with hash map and equality which only considers goal point
  std::unordered_map<FrontierPtr, bool, FrontierHash,
    FrontierGoalPointEquality> frontier_blacklist_;
  std::mutex blacklist_mutex_;

  double min_traversable_distance = std::numeric_limits<double>::max();
  double max_traversable_distance = -1.0 * std::numeric_limits<double>::max();
  std::shared_ptr<BasePlanner> planner_plugin_;

  double min_arrival_info_per_frontier = std::numeric_limits<double>::max();
  double max_arrival_info_per_frontier = -1.0 * std::numeric_limits<double>::max();
  std::shared_ptr<BaseInformationGain> information_gain_plugin_;

};
}
#endif
