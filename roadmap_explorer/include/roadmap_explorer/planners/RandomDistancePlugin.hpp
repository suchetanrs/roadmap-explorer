#ifndef RANDOM_DISTANCE_PLUGIN_HPP_
#define RANDOM_DISTANCE_PLUGIN_HPP_

#include "roadmap_explorer/planners/BasePlanner.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

class RandomDistancePlugin : public BasePlanner
{
public:
  RandomDistancePlugin() = default;

  ~RandomDistancePlugin() override = default;

  void configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) override;

  void reset() override;

  double getRandomVal();

  void setPlanForFrontier(
    const geometry_msgs::msg::Pose start_pose_w,
    FrontierPtr & goal_point_w) override;
};

}  // namespace roadmap_explorer

#endif  // RANDOM_DISTANCE_PLUGIN_HPP_
