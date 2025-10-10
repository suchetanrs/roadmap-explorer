#include "roadmap_explorer/bt_plugins/OptimizeFullPathPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class OptimizeFullPathBT : public BT::SyncActionNode
    {
    public:
    OptimizeFullPathBT(
        const std::string & name, const BT::NodeConfiguration & config,
        std::shared_ptr<FullPathOptimizer> full_path_optimizer,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
        std::shared_ptr<nav2_util::LifecycleNode> node)
    : BT::SyncActionNode(name, config)
    {
        explore_costmap_ros_ = explore_costmap_ros;
        full_path_optimizer_ = full_path_optimizer;
        node_ = node;
        LOG_INFO("OptimizeFullPathBT Constructor");
    }

    void blacklistFrontier(const FrontierPtr & frontier, BT::Blackboard::Ptr blackboard)
    {
        auto blacklisted_frontiers = blackboard->get<std::shared_ptr<std::vector<FrontierPtr>>>(
            "blacklisted_frontiers");
        blacklisted_frontiers->push_back(frontier);
    }

    BT::NodeStatus tick() override
    {
        EventLoggerInstance.startEvent("OptimizeFullPathBT");
        LOG_FLOW("MODULE OptimizeFullPathBT");
        std::vector<FrontierPtr> globalFrontierList;
        getInput<std::vector<FrontierPtr>>("frontier_costs_result", globalFrontierList);
        geometry_msgs::msg::PoseStamped robotP;
        geometry_msgs::msg::PoseStamped robotP3D;
        if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP)) {
        // Handle the case when "latest_robot_pose" is not found
        LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
        throw RoadmapExplorerException("Failed to retrieve latest_robot_pose from blackboard.");
        }
        FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
        auto return_state = full_path_optimizer_->getNextGoal(
        globalFrontierList, allocatedFrontier, robotP);
        if (return_state) {
        setOutput<FrontierPtr>("allocated_frontier", allocatedFrontier);
        } else {
        LOG_INFO("Full path optimization failed.");
        EventLoggerInstance.endEvent("OptimizeFullPathBT", 0);
        config().blackboard->set<ExplorationErrorCode>(
            "error_code_id", ExplorationErrorCode::FULL_PATH_OPTIMIZATION_FAILURE);
        return BT::NodeStatus::FAILURE;
        }
        EventLoggerInstance.endEvent("OptimizeFullPathBT", 0);
        setOutput<FrontierPtr>("allocated_frontier", allocatedFrontier);

        nav_msgs::msg::Path path;
        if (!full_path_optimizer_->refineAndPublishPath(robotP, allocatedFrontier, path)) {
        LOG_ERROR(
            "Failed to refine and publish path between robotP: " << robotP.pose.position.x << ", " << robotP.pose.position.y << " and " <<
            allocatedFrontier);
        config().blackboard->set<ExplorationErrorCode>(
            "error_code_id", ExplorationErrorCode::REFINED_PATH_COMPUTATION_FAILURE);
        blacklistFrontier(allocatedFrontier, config().blackboard);
        return BT::NodeStatus::FAILURE;
        }
        setOutput<nav_msgs::msg::Path>("optimized_path", path);
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::vector<FrontierPtr>>("frontier_costs_result"),
        BT::OutputPort<FrontierPtr>("allocated_frontier"),
        BT::OutputPort<nav_msgs::msg::Path>("optimized_path")};
    }

    std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    std::shared_ptr<nav2_util::LifecycleNode> node_;
    };

    OptimizeFullPathPlugin::OptimizeFullPathPlugin()
    {
    }

    OptimizeFullPathPlugin::~OptimizeFullPathPlugin()
    {
    }

    void OptimizeFullPathPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<OptimizeFullPathBT>(
                name, 
                config, 
                context->full_path_optimizer,
                context->explore_costmap_ros,
                context->node);
        };
        factory.registerBuilder<OptimizeFullPathBT>("OptimizeFullPath", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::OptimizeFullPathPlugin,
    roadmap_explorer::BTPlugin)
