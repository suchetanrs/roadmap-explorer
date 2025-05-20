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
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/FrontierSearch.hpp"
#include "roadmap_explorer/util/GeneralUtils.hpp"
#include "roadmap_explorer/CostCalculator.hpp"

namespace roadmap_explorer
{

struct GetFrontierCostsRequest
{
  geometry_msgs::msg::PoseStamped start_pose;
  std::vector<FrontierPtr> frontier_list;
  std::vector<std::vector<double>> every_frontier;
  std::vector<FrontierPtr> prohibited_frontiers;
};

struct GetFrontierCostsResponse
{
  bool success;
  std::vector<FrontierPtr> frontier_list;
  std::vector<double> frontier_distances;
  std::vector<double> frontier_arrival_information;
  std::vector<double> frontier_path_information;
};

class CostAssigner
{
public:
  CostAssigner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

  ~CostAssigner();

  bool updateBoundaryPolygon(geometry_msgs::msg::PolygonStamped & explore_boundary);

  bool getFrontierCosts(
    std::shared_ptr<GetFrontierCostsRequest> requestData,
    std::shared_ptr<GetFrontierCostsResponse> resultData);

protected:
  bool processChosenApproach(
    std::vector<FrontierPtr> & frontier_list,
    geometry_msgs::msg::Pose & start_pose_w);

  /**
       * @param costTypes can take the values
       * "ArrivalInformation",
       * "A*PlannerDistance" OR "EuclideanDistance" OR "RoadmapPlannerDistance",
       *
       * "RandomCosts",
       *
       * "ClosestFrontier"
       */
  bool assignCosts(
    std::vector<FrontierPtr> & frontier_list, std::vector<double> polygon_xy_min_max,
    geometry_msgs::msg::Pose start_pose_w, std::vector<std::vector<std::string>> & costTypes);

  void setFrontierBlacklist(std::vector<FrontierPtr> & blacklist);

private:
  geometry_msgs::msg::Polygon polygon_;
  std::vector<double> polygon_xy_min_max_;

  bool planner_allow_unknown_;
  double frontierDetectRadius_;
  bool add_heading_cost_;

  nav2_costmap_2d::LayeredCostmap * layered_costmap_;
  nav2_costmap_2d::Costmap2D * costmap_;

  std::shared_ptr<FrontierCostCalculator> costCalculator_;

  // std::shared_ptr<RosVisualizer> RosVisualizer_;
  std::unordered_map<FrontierPtr, bool, FrontierHash,
    FrontierGoalPointEquality> frontier_blacklist_;
  std::mutex blacklist_mutex_;
};
}
#endif
