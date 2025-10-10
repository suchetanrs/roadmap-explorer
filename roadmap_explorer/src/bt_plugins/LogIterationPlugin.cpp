#include "roadmap_explorer/bt_plugins/LogIterationPlugin.hpp"
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

    void LogIteration::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder_log_iteration =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<LogIterationBT>(name, config);
        };
        factory.registerBuilder<LogIterationBT>("LogIterationBT", builder_log_iteration);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::LogIteration,
    roadmap_explorer::BTPlugin)