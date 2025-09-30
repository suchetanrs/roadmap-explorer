#ifndef EUCLIDEAN_PLUGIN_HPP_
#define EUCLIDEAN_PLUGIN_HPP_

#include "roadmap_explorer/planners/BasePlanner.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

class EuclideanDistancePlugin : public BasePlanner
{
public:
  EuclideanDistancePlugin() = default;

  ~EuclideanDistancePlugin() override = default;

  void configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) override;

  void reset() override;

  void setPlanForFrontier(
    const geometry_msgs::msg::Pose start_pose_w,
    FrontierPtr & goal_point_w) override;
};

}  // namespace roadmap_explorer

#endif  // EUCLIDEAN_PLUGIN_HPP_
