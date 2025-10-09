#ifndef BASE_PLANNER_HPP_
#define BASE_PLANNER_HPP_

#include <memory>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/RosVisualizer.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

/**
 * @brief Base class for path planning algorithms
 *
 * This abstract base class defines the interface that all path planning
 * algorithms must implement. It provides a plugin-based architecture for
 * different planning strategies used in frontier exploration.
 */
class BasePlanner
{
public:
  /**
   * @brief Default constructor
   */
  BasePlanner() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~BasePlanner() = default;

  /**
   * @brief Configure the planner with necessary components
   * @param explore_costmap_ros Shared pointer to the costmap ROS wrapper
   */
  virtual void configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) = 0;

  /**
   * @brief Reset the planner state
   *
   * Clears any internal state or cached data from previous planning operations.
   */
  virtual void reset() = 0;

  /**
   * @brief Set planning parameters specific to the planner. Run everytime you want to update the parameters.
   */
  virtual void updateParameters()
  {
    max_planning_distance_ = parameterInstance.getValue<double>(
      "costCalculator.max_planning_distance_roadmap");
    closeness_rejection_threshold_ = parameterInstance.getValue<double>(
      "costCalculator.closeness_rejection_threshold");
    planner_allow_unknown_ =
      parameterInstance.getValue<bool>("costCalculator.planner_allow_unknown");
  }

  /**
   * @brief Plan a path from start pose to frontier goal
   * @param start_pose_w Starting pose in world coordinates
   * @param goal_frontier Target frontier to plan to
   */
  virtual void setPlanForFrontier(
    const geometry_msgs::msg::Pose start_pose_w,
    FrontierPtr & goal_point_w) = 0;

protected:
  nav2_costmap_2d::Costmap2D * exploration_costmap_;
  double closeness_rejection_threshold_ = 0.5;
  double max_planning_distance_ = 50.0;
  bool planner_allow_unknown_ = false;
};

}  // namespace roadmap_explorer

#endif  // BASE_PLANNER_HPP_


