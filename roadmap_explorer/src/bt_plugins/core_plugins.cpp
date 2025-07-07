#include "roadmap_explorer/bt_plugins/core_plugins.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace roadmap_explorer
{
    class LogIterationBT : public BT::SyncActionNode
    {
    public:
        LogIterationBT(
            const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config)
        {
            LOG_INFO("LogIterationBT Constructor");
        }

        BT::NodeStatus tick() override
        {
            EventLoggerInstance.incrementPlanningCount();
            LOG_FLOW("MODULE LogIterationBT");
            return BT::NodeStatus::SUCCESS;
        }
    };

    LogIteration::LogIteration()
    {
    }

    LogIteration::~LogIteration()
    {
    }

    void LogIteration::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
    {
        (void)explore_costmap_ros;
        (void)node;
        BT::NodeBuilder builder_log_iteration =
            [&](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<LogIterationBT>(name, config);
        };
        factory.registerBuilder<LogIterationBT>("LogIterationBT", builder_log_iteration);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::LogIteration,
    roadmap_explorer::BTPlugin)