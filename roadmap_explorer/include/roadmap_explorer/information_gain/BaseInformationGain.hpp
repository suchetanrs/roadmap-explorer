#ifndef BASE_INFORMATION_GAIN_HPP_
#define BASE_INFORMATION_GAIN_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

/**
 * @brief Base class for information gain calculation algorithms
 *
 * This abstract base class defines the interface that all information gain
 * calculation algorithms must implement. It provides a plugin-based architecture
 * for different strategies to compute the expected information gain from visiting
 * a frontier position.
 */
class BaseInformationGain
{
public:
  /**
   * @brief Default constructor
   */
  BaseInformationGain() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~BaseInformationGain() = default;

  /**
   * @brief Configure the information gain calculator with necessary components
   * @param explore_costmap_ros Shared pointer to the costmap ROS wrapper
   */
  virtual void configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node) = 0;

  /**
   * @brief Reset the information gain calculator state
   *
   * Clears any internal state or cached data from previous calculations.
   */
  virtual void reset() = 0;

  /**
   * @brief Calculate and set the information gain for a frontier
   *
   * This method computes the expected information gain from visiting the given
   * frontier and sets the information gain and optimal orientation.
   *
   * @param frontier The frontier to calculate information gain for
   * @param polygon_xy_min_max Boundary polygon limits [x_min, y_min, x_max, y_max]
   */
  virtual void setInformationGainForFrontier(
    FrontierPtr & frontier,
    std::vector<double> & polygon_xy_min_max) = 0;

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_ = nullptr;
  std::shared_ptr<nav2_util::LifecycleNode> node_ = nullptr;
};

}  // namespace roadmap_explorer

#endif  // BASE_INFORMATION_GAIN_HPP_
