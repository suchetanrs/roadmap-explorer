#include <vector>
#include <algorithm>
#include <limits>
#include "roadmap_explorer/Frontier.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <roadmap_explorer/util/GeometryUtils.hpp>
#include <roadmap_explorer/planners/FrontierRoadmap.hpp>

const double ARRIVAL_INFORMATION_THRESHOLD = 70.0;
const double NUM_FRONTIERS_IN_LOCAL_AREA = 5.0;
const double DISTANCE_THRESHOLD_GLOBAL_CLUSTER = 5.0;
const double CLUSTER_PADDING = 0.25;
const double LOCAL_FRONTIER_SEARCH_RADIUS = 12.0; // 6.0 in m
const bool ADD_YAW_TO_TSP = false;
const bool ADD_DISTANCE_TO_ROBOT_TO_TSP = false;

const double BLACKLISTING_CIRCLE_RADIUS = 1.7; // in m
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
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

  // new
  void addToMarkerArrayLinePolygon(
    visualization_msgs::msg::MarkerArray & marker_array, std::vector<FrontierPtr> & frontier_list,
    std::string ns, float r, float g, float b, int id);

  void addToMarkerArraySolidPolygon(
    visualization_msgs::msg::MarkerArray & marker_array,
    geometry_msgs::msg::Point center, double radius, std::string ns,
    float r, float g, float b, int id);

  double calculateLengthRobotToGoal(
    const FrontierPtr & robot, const FrontierPtr & goal,
    geometry_msgs::msg::PoseStamped & robotP);

  double calculatePathLength(std::vector<FrontierPtr> & path);

  bool getBestFullPath(
    SortedFrontiers & sortedFrontiers, std::vector<FrontierPtr> & bestPath,
    geometry_msgs::msg::PoseStamped & robotP);

  bool prepareGlobalOptimization(
    SortedFrontiers & sortedFrontiers,
    std::vector<FrontierPtr> & bestPath,
    geometry_msgs::msg::PoseStamped & robotP);

  void getFilteredFrontiersN(
    std::vector<FrontierPtr> & frontier_list, size_t n,
    SortedFrontiers & sortedFrontiers,
    geometry_msgs::msg::PoseStamped & robotP);

  void getFilteredFrontiers(
    std::vector<FrontierPtr> & frontier_list,
    SortedFrontiers & sortedFrontiers,
    geometry_msgs::msg::PoseStamped & robotP);

  bool getNextGoal(
    std::vector<FrontierPtr> & frontier_list, FrontierPtr & nextFrontier, size_t n,
    geometry_msgs::msg::PoseStamped & robotP,
    geometry_msgs::msg::PoseStamped & robotPFI);

  bool refineAndPublishPath(geometry_msgs::msg::PoseStamped & robotP, FrontierPtr & goalFrontier);

  void clearPlanCache()
  {
    frontier_pair_distances_.clear();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_search_area_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontier_nav2_plan_;
  std::unordered_map<FrontierPair, RoadmapPlanResult, FrontierPairHash> frontier_pair_distances_;
};
}
