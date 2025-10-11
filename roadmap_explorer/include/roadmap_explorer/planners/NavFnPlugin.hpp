#ifndef NAV_FN_PLUGIN_HPP_
#define NAV_FN_PLUGIN_HPP_

#include "roadmap_explorer/planners/BasePlanner.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Parameters.hpp"

#include "roadmap_explorer/planners/NavFn.hpp"

namespace roadmap_explorer
{

class PluginNavFn : public BasePlanner
{
public:
  PluginNavFn() = default;

  ~PluginNavFn() override = default;

  void configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node) override;

  void reset() override;

  void setPlanForFrontier(
    const geometry_msgs::msg::Pose start_pose_w,
    FrontierPtr & goal_point_w) override;

private:
  nav2_costmap_2d::Costmap2D * exploration_costmap_ = nullptr;
};

}  // namespace roadmap_explorer

#endif  // NAV_FN_PLUGIN_HPP_
