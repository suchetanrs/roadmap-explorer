// plugin_interface.hpp
#pragma once

#ifdef ROS_DISTRO_HUMBLE
  #include <behaviortree_cpp_v3/bt_factory.h>
#elif ROS_DISTRO_JAZZY
  #include <behaviortree_cpp/bt_factory.h>
#else
  #error "Unsupported ROS distro"
#endif

namespace roadmap_explorer
{

class BTPlugin
{
  public:
  virtual ~BTPlugin() = default;

  virtual void registerNodes(BT::BehaviorTreeFactory & factory) = 0;
};

}  // namespace roadmap_explorer