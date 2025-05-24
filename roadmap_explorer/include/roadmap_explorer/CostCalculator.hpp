#ifndef COST_CALCULATOR_HPP_
#define COST_CALCULATOR_HPP_

#include <algorithm>
#include <fstream>
#include <thread>
#include <random>
#include <unordered_map>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav2_navfn_planner/navfn.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/RosVisualizer.hpp"
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/planners/FrontierRoadmap.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

class FrontierCostCalculator
{
public:
  FrontierCostCalculator(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

  // ----------------Arrival information related--------------------
  // sets frontiers as not achievable if frontier information is too low.
  // also sets frontiers as not achievable if the robot footprint radius centered at the frontier is in obstacle
  void setArrivalInformationForFrontier(
    FrontierPtr & goal_point_w,
    std::vector<double> & polygon_xy_min_max);

  double setArrivalInformationLimits();

  // ----------------Planning related--------------------
  // all of these set frontiers as not achievable if the frontier path distance is less than closeness_rejection_threshold_

  // Navfn A* planner
  void setPlanForFrontier(
    geometry_msgs::msg::Pose start_pose_w, FrontierPtr & goal_point_w);

  void setPlanForFrontierRoadmap(
    geometry_msgs::msg::Pose start_pose_w, FrontierPtr & goal_point_w);

  void setPlanForFrontierEuclidean(
    geometry_msgs::msg::Pose start_pose_w,
    FrontierPtr & goal_point_w);

  // -----------------Random costs--------------
  void setRandomMetaData(FrontierPtr & goal_point_w);

  // -----------------For closest frontier implementation-----------------

  void setClosestFrontierMetaData(
    geometry_msgs::msg::Pose start_pose_w, FrontierPtr & goal_point_w);

  // --------------Other----------------
  void recomputeNormalizationFactors(FrontierPtr & frontier);

  void prepareForCostCalculation()
  {
    min_traversable_distance = std::numeric_limits<double>::max();
    max_traversable_distance = -1.0 * std::numeric_limits<double>::max();
    min_arrival_info_per_frontier = std::numeric_limits<double>::max();
    max_arrival_info_per_frontier = -1.0 * std::numeric_limits<double>::max();

    // these are set here again because we need to update get the latest values of parameters before starting the cost calculation
    MAX_CAMERA_DEPTH = parameterInstance.getValue<double>("costCalculator.max_camera_depth");
    DELTA_THETA = parameterInstance.getValue<double>("costCalculator.delta_theta");
    CAMERA_FOV = parameterInstance.getValue<double>("costCalculator.camera_fov");
    factor_of_max_is_min = parameterInstance.getValue<double>("costCalculator.factor_of_max_is_min");
    closeness_rejection_threshold_ = parameterInstance.getValue<double>("costCalculator.closeness_rejection_threshold");
    planner_allow_unknown_ = parameterInstance.getValue<bool>("costCalculator.planner_allow_unknown");
  }

private:
  double getRandomVal();

  double getMinPlanDistance()
  {
    return min_traversable_distance;
  }

  double getMaxPlanDistance()
  {
    return max_traversable_distance;
  }

  double getMinArrivalInformation()
  {
    return min_arrival_info_gt_;
  }

  double getMaxArrivalInformation()
  {
    return max_arrival_info_gt_;
  }

  nav2_costmap_2d::Costmap2D * exploration_costmap_;
  // std::shared_ptr<RosVisualizer> RosVisualizer_;
  double min_traversable_distance = std::numeric_limits<double>::max();
  double max_traversable_distance = -1.0 * std::numeric_limits<double>::max();
  double min_arrival_info_per_frontier = std::numeric_limits<double>::max();
  double max_arrival_info_per_frontier = -1.0 * std::numeric_limits<double>::max();
  double robot_radius_;
  double max_arrival_info_gt_ = -1.0 * std::numeric_limits<double>::max();
  double min_arrival_info_gt_ = std::numeric_limits<double>::max();
  bool arrival_info_limits_set_ = false;
  bool planner_allow_unknown_;

  double MAX_CAMERA_DEPTH;
  double DELTA_THETA;
  double CAMERA_FOV;
  double factor_of_max_is_min = 0.70;
  double closeness_rejection_threshold_ = 0.5; // meters
};
}

#endif
