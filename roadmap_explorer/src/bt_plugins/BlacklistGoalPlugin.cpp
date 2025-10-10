#include "roadmap_explorer/bt_plugins/BlacklistGoalPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class BlacklistGoal : public BT::SyncActionNode
    {
    public:
    BlacklistGoal(
        const std::string & name, const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
    {
        LOG_INFO("BlacklistGoal Constructor");
    }

    void blacklistFrontier(const FrontierPtr & frontier, BT::Blackboard::Ptr blackboard)
    {
        auto blacklisted_frontiers = blackboard->get<std::shared_ptr<std::vector<FrontierPtr>>>(
            "blacklisted_frontiers");
        blacklisted_frontiers->push_back(frontier);
    }

    BT::NodeStatus tick() override
    {
        EventLoggerInstance.startEvent("BlacklistGoal");
        LOG_FLOW("MODULE BlacklistGoal");
        FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
        if(!config().blackboard->get<FrontierPtr>("latest_failed_frontier", allocatedFrontier))
        {
        throw RoadmapExplorerException("Could not get latest allocated frontier from blackboard");
        }
        LOG_WARN("Setting blacklist " << allocatedFrontier);
        blacklistFrontier(allocatedFrontier, config().blackboard);
        return BT::NodeStatus::SUCCESS;
    }
    };

    BlacklistGoalPlugin::BlacklistGoalPlugin()
    {
    }

    BlacklistGoalPlugin::~BlacklistGoalPlugin()
    {
    }

    void BlacklistGoalPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<BlacklistGoal>(
                name, 
                config);
        };
        factory.registerBuilder<BlacklistGoal>("BlacklistGoal", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::BlacklistGoalPlugin,
    roadmap_explorer::BTPlugin)
