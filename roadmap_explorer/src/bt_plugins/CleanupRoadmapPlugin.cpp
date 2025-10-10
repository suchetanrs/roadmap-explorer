#include "roadmap_explorer/bt_plugins/CleanupRoadmapPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class CleanupRoadMapBT : public BT::SyncActionNode
    {
    public:
    CleanupRoadMapBT(
        const std::string & name, const BT::NodeConfiguration & config,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
        std::shared_ptr<FullPathOptimizer> full_path_optimizer)
    : BT::SyncActionNode(name, config)
    {
        full_path_optimizer_ = full_path_optimizer;
        explore_costmap_ros_ = explore_costmap_ros;
        LOG_INFO("CleanupRoadMapBT Constructor");
    }

    BT::NodeStatus tick() override
    {
        LOG_FLOW("MODULE CleanupRoadMapBT");
        // LOG_INFO("Time since last clearance: " << EventLoggerInstance.getTimeSinceStart("clearRoadmap"));
        double time_between_cleanup;
        getInput("time_between_cleanup", time_between_cleanup);
        if (EventLoggerInstance.getTimeSinceStart("clearRoadmap") < time_between_cleanup) {
        return BT::NodeStatus::SUCCESS;
        }
        EventLoggerInstance.startEvent("clearRoadmap");
        EventLoggerInstance.startEvent("CleanupRoadMapBT");
        EventLoggerInstance.startEvent("roadmapReconstructionFull");
        bool correct_loop_closure_;
        getInput("correct_loop_closure", correct_loop_closure_);
        LOG_WARN("Reconstructing roadmap and clearing plan cache!");
        frontierRoadmapInstance.reConstructGraph(false, correct_loop_closure_);
        EventLoggerInstance.endEvent("roadmapReconstructionFull", 1);
        full_path_optimizer_->clearPlanCache();
        // TODO: make sure to add a thing such that the entire roadmap within a certain distance (max frontier search distance) is reconstructed periodically
        EventLoggerInstance.endEvent("CleanupRoadMapBT", 0);
        // frontierRoadmapInstance.countTotalItemsInSpatialMap();
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {
        BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
        BT::InputPort<bool>("correct_loop_closure"),
        BT::InputPort<double>("time_between_cleanup")};
    }

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
    };

    CleanupRoadmapPlugin::CleanupRoadmapPlugin()
    {
    }

    CleanupRoadmapPlugin::~CleanupRoadmapPlugin()
    {
    }

    void CleanupRoadmapPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<CleanupRoadMapBT>(
                name, 
                config,
                context->explore_costmap_ros,
                context->full_path_optimizer);
        };
        factory.registerBuilder<CleanupRoadMapBT>("CleanupRoadmap", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::CleanupRoadmapPlugin,
    roadmap_explorer::BTPlugin)
