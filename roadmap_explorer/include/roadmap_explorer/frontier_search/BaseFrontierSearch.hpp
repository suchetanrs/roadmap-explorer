#ifndef FRONTIER_SEARCH_BASE_HPP_
#define FRONTIER_SEARCH_BASE_HPP_

#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "roadmap_explorer/Frontier.hpp"

namespace roadmap_explorer
{

/**
 * @brief Enumeration of possible results from frontier search operations
 */
enum class FrontierSearchResult
{
  ROBOT_OUT_OF_BOUNDS = 0,        ///< Robot position is outside the costmap bounds
  CANNOT_FIND_CELL_TO_SEARCH = 1, ///< No valid cells found to start the search
  SUCCESSFUL_SEARCH = 2,           ///< Search completed successfully
  NO_FRONTIERS_FOUND = 3
};

/**
 * @brief Base class for frontier search algorithms
 *
 * This abstract base class defines the interface that all frontier search
 * algorithms must implement. It provides a plugin-based architecture for
 * different frontier detection strategies.
 */
class FrontierSearchBase
{
public:
  /**
   * @brief Default constructor
   */
  FrontierSearchBase() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~FrontierSearchBase() = default;

  /**
   * @brief Configure the frontier search with a costmap
   * @param costmap Pointer to the costmap for frontier detection.
   * @param node Shared pointer to the lifecycle node for parameter management
   */
  virtual void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node) = 0;

  /**
   * @brief Reset the frontier search state
   *
   * Clears any internal state or cached data from previous searches.
   */
  virtual void reset() = 0;

  /**
   * @brief Search for frontiers from a given position
   * @param position The starting position for the search in world coordinates
   * @param output_frontier_list Output vector to store found "clustered" frontiers (will be cleared and populated)
   * @return Result of the search operation indicating success or failure reason
   */
  virtual FrontierSearchResult searchFrom(
    geometry_msgs::msg::Point position,
    std::vector<FrontierPtr> & output_frontier_list,
    double max_frontier_search_distance) = 0;

  /**
   * @brief Get all frontiers found in the last search operation
   * @return Vector of all frontier points as [x, y] coordinates in world frame
   * @note This method should only be called after a successful searchFrom() operation
   */
  virtual std::vector<std::vector<double>> getAllFrontiers() = 0;

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_ = nullptr; ///< Pointer to the costmap used for frontier detection
  std::shared_ptr<nav2_util::LifecycleNode> node_ = nullptr; ///< Shared pointer to the lifecycle node
};

}  // namespace roadmap_explorer

#endif  // FRONTIER_SEARCH_BASE_HPP_
