#ifndef FRONTIER_ROADMAP_PLUGIN_HPP_
#define FRONTIER_ROADMAP_PLUGIN_HPP_

#include "roadmap_explorer/planners/BasePlanner.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Parameters.hpp"

#include "roadmap_explorer/planners/FrontierRoadmap.hpp"

namespace roadmap_explorer
{

class PluginFrontierRoadmap : public BasePlanner
{
public:
  PluginFrontierRoadmap() = default;

  ~PluginFrontierRoadmap() override = default;

  void configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) override;

  void reset() override;

  void setPlanForFrontier(
    const geometry_msgs::msg::Pose start_pose_w,
    FrontierPtr & goal_point_w) override;
};

}  // namespace roadmap_explorer

#endif  // FRONTIER_ROADMAP_PLUGIN_HPP_
