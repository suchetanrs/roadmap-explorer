/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#include "roadmap_explorer/bt_plugins/ProcessFrontierCostsPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class ProcessFrontierCostsBT : public BT::SyncActionNode
    {
    public:
    ProcessFrontierCostsBT(
        const std::string & name, const BT::NodeConfiguration & config,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
        std::shared_ptr<CostAssigner> cost_assigner_ptr,
        std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
    : BT::SyncActionNode(name, config)
    {
        explore_costmap_ros_ = explore_costmap_ros;
        cost_assigner_ptr_ = cost_assigner_ptr;
        ros_node_ptr_ = ros_node_ptr;
        LOG_INFO("ProcessFrontierCostsBT Constructor");
    }

    BT::NodeStatus tick() override
    {
        EventLoggerInstance.startEvent("ProcessFrontierCosts");
        LOG_FLOW("MODULE ProcessFrontierCostsBT");
        auto frontierCostsRequestPtr = std::make_shared<CalculateFrontierCostsRequest>();

        frontierCostsRequestPtr->prohibited_frontiers =
        *(config().blackboard->get<std::shared_ptr<std::vector<FrontierPtr>>>(
            "blacklisted_frontiers"));

        if (!getInput<std::vector<FrontierPtr>>(
            "frontier_list",
            frontierCostsRequestPtr->frontier_list))
        {
        BT::RuntimeError("No correct input recieved for frontier list");
        }
        if (!getInput<std::vector<std::vector<double>>>(
            "every_frontier",
            frontierCostsRequestPtr->every_frontier))
        {
        BT::RuntimeError("No correct input recieved for every_frontier");
        }
        if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>(
            "latest_robot_pose",
            frontierCostsRequestPtr->start_pose))
        {
        // Handle the case when "latest_robot_pose" is not found
        LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
        throw RoadmapExplorerException("Failed to retrieve latest_robot_pose from blackboard.");
        }
        LOG_INFO("Request to get frontier costs sent");
        bool frontierCostsSuccess = cost_assigner_ptr_->populateFrontierCosts(
        frontierCostsRequestPtr);
        if (frontierCostsSuccess == false) {
        LOG_INFO("Failed to receive response for getNextFrontier called from within the robot.");
        EventLoggerInstance.endEvent("ProcessFrontierCosts", 0);
        config().blackboard->set<ExplorationErrorCode>(
            "error_code_id",
            ExplorationErrorCode::COST_COMPUTATION_FAILURE);
        return BT::NodeStatus::FAILURE;
        }
        bool atleast_one_achievable_frontier = false;
        for (auto & fip : frontierCostsRequestPtr->frontier_list) {
        if (fip->isAchievable()) {
            atleast_one_achievable_frontier = true;
            break;
        }
        }
        if (!atleast_one_achievable_frontier) {
        LOG_INFO("No achievable frontiers left. Exiting.");
        config().blackboard->set<ExplorationErrorCode>(
            "error_code_id", ExplorationErrorCode::NO_ACHIEVABLE_FRONTIERS_LEFT);
        return BT::NodeStatus::FAILURE;
        }
        setOutput("frontier_costs_result", frontierCostsRequestPtr->frontier_list);
        EventLoggerInstance.endEvent("ProcessFrontierCosts", 0);
        rosVisualizerInstance.visualizeFrontierMarkers("frontier_cell_markers", frontierCostsRequestPtr->frontier_list);
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
        BT::InputPort<std::vector<std::vector<double>>>("every_frontier"),
        BT::OutputPort<std::vector<FrontierPtr>>("frontier_costs_result")};
    }

    std::shared_ptr<FrontierSearchBase> frontierSearchPtr_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    std::shared_ptr<CostAssigner> cost_assigner_ptr_;
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
    };
        

    ProcessFrontierCostsPlugin::ProcessFrontierCostsPlugin()
    {
    }

    ProcessFrontierCostsPlugin::~ProcessFrontierCostsPlugin()
    {
    }

    void ProcessFrontierCostsPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<ProcessFrontierCostsBT>(
                name, 
                config, 
                context->explore_costmap_ros,
                context->cost_assigner,
                context->node);
        };
        factory.registerBuilder<ProcessFrontierCostsBT>("ProcessFrontierCosts", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::ProcessFrontierCostsPlugin,
    roadmap_explorer::BTPlugin)
