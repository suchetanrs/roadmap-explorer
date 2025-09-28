#ifndef FULL_PATH_OPTIMIZER_HPP_
#define FULL_PATH_OPTIMIZER_HPP_

#include <vector>
#include <algorithm>
#include <limits>

#include <visualization_msgs/msg/marker_array.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "roadmap_explorer/util/GeometryUtils.hpp"
#include "roadmap_explorer/planners/FrontierRoadmap.hpp"
#include "roadmap_explorer/planners/NavFn.hpp"
#include "roadmap_explorer/planners/ThetaStar.hpp"


#include "roadmap_explorer/Parameters.hpp"
#include "roadmap_explorer/Frontier.hpp"
namespace roadmap_explorer
{

struct FrontierPair
{
  // Constructor
  FrontierPair(FrontierPtr f1_, FrontierPtr f2_)
  : f1(f1_), f2(f2_) {}

  FrontierPtr f1;
  FrontierPtr f2;

  // Custom operator< for ordering points in the map
  bool operator<(const FrontierPair & other) const
  {
    // Compare f1 and f2 in lexicographical order
    if (f1->getUID() != other.f1->getUID()) {
      return f1->getUID() < other.f1->getUID();
    }
    return f2->getUID() < other.f2->getUID();
  }

  bool operator==(const FrontierPair & other) const
  {
    // Compare f1 and f2 in lexicographical order
    return f2->getGoalPoint() == other.f2->getGoalPoint() &&
           f1->getGoalPoint() == other.f1->getGoalPoint();
  }
};

// Custom hash function for FrontierPair
struct FrontierPairHash
{
  std::size_t operator()(const FrontierPair & fp) const
  {
    // Combine the hash of both FrontierPtr objects
    std::size_t h1 = std::hash<int>{}(fp.f1->getUID());
    std::size_t h2 = std::hash<int>{}(fp.f2->getUID());

    // Hash combination technique (can vary)
    return h1 ^ (h2 << 1);         // Example of hash combining
  }
};

struct SortedFrontiers
{
  std::vector<FrontierPtr> local_frontiers;
  std::vector<FrontierPtr> global_frontiers;
  FrontierPtr closest_global_frontier;
};

class FullPathOptimizer
{
public:
  FullPathOptimizer(
    std::shared_ptr<nav2_util::LifecycleNode> node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

  ~FullPathOptimizer();

  bool getNextGoal(
    std::vector<FrontierPtr> & frontier_list, FrontierPtr & nextFrontier,
    geometry_msgs::msg::PoseStamped & robotP);

  void clearPlanCache()
  {
    frontier_pair_distances_.clear();
  }

  double calculateLengthRobotToGoal(
    const FrontierPtr & robot, const FrontierPtr & goal);

  bool refineAndPublishPath(
    geometry_msgs::msg::PoseStamped & robotP, FrontierPtr & goalFrontier,
    nav_msgs::msg::Path & refined_path);

private:
  void addToMarkerArraySolidPolygon(
    visualization_msgs::msg::MarkerArray & marker_array,
    geometry_msgs::msg::Point center, double radius, std::string ns,
    float r, float g, float b, int id);

  double calculatePathLength(std::vector<FrontierPtr> & path);

  void getFilteredFrontiers(
    std::vector<FrontierPtr> & frontier_list,
    SortedFrontiers & sortedFrontiers,
    geometry_msgs::msg::PoseStamped & robotP);

  void getFilteredFrontiersN(
    std::vector<FrontierPtr> & frontier_list, int n,
    SortedFrontiers & sortedFrontiers,
    geometry_msgs::msg::PoseStamped & robotP);

  bool getBestFullPath(
    SortedFrontiers & sortedFrontiers, std::vector<FrontierPtr> & bestPath,
    geometry_msgs::msg::PoseStamped & robotP);

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    local_search_area_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr frontier_nav2_plan_;
  std::unordered_map<FrontierPair, RoadmapPlanResult, FrontierPairHash> frontier_pair_distances_;

  double num_frontiers_in_local_area = 5.0;
  double local_frontier_search_radius = 12.0; // 6.0 in m
  bool add_yaw_to_tsp = false;
  bool add_distance_to_robot_to_tsp = false;
};
}

#endif
