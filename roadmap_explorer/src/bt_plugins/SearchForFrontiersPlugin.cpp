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

#include "roadmap_explorer/bt_plugins/SearchForFrontiersPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class SearchForFrontiersBT : public BT::SyncActionNode
    {
    public:
    SearchForFrontiersBT(
        const std::string & name, const BT::NodeConfiguration & config,
        std::shared_ptr<FrontierSearchBase> frontierSearchPtr,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
        std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
    : BT::SyncActionNode(name, config)
    {
        explore_costmap_ros_ = explore_costmap_ros;
        frontierSearchPtr_ = frontierSearchPtr;
        ros_node_ptr_ = ros_node_ptr;
        LOG_DEBUG("SearchForFrontiersBT Constructor");
    }

    BT::NodeStatus tick() override
    {
        LOG_FLOW("MODULE SearchForFrontiersBT");
        EventLoggerInstance.startEvent("SearchForFrontiers");
        frontierSearchPtr_->reset();
        explore_costmap_ros_->getCostmap()->getMutex()->lock();
        LOG_DEBUG("SearchForFrontiersBT OnStart called ");
        geometry_msgs::msg::PoseStamped robotP;
        explore_costmap_ros_->getRobotPose(robotP);
        config().blackboard->set<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
        LOG_INFO("Using robot pose: " << robotP.pose.position.x << ", " << robotP.pose.position.y);
        std::vector<FrontierPtr> frontier_list;
        auto current_frontier_search_distance = config().blackboard->get<double>("current_frontier_search_distance");
        LOG_INFO("Current frontier search distance: " << current_frontier_search_distance);
        auto searchResult = frontierSearchPtr_->searchFrom(robotP.pose.position, frontier_list, current_frontier_search_distance);
        if (searchResult != FrontierSearchResult::SUCCESSFUL_SEARCH)
        {
            LOG_INFO("No frontiers in current search radius.")
            explore_costmap_ros_->getCostmap()->getMutex()->unlock();
            config().blackboard->set<ExplorationErrorCode>(
                "error_code_id",
                ExplorationErrorCode::NO_FRONTIERS_IN_CURRENT_RADIUS);
            return BT::NodeStatus::FAILURE;
        }
        auto every_frontier = frontierSearchPtr_->getAllFrontiers();
        LOG_INFO("Recieved " << frontier_list.size() << " frontiers");
        setOutput("frontier_list", frontier_list);
        setOutput("every_frontier", every_frontier);
        LOG_DEBUG("Set frontier outputs");
        rosVisualizerInstance.visualizePointCloud("frontiers", frontier_list, explore_costmap_ros_->getLayeredCostmap()->getGlobalFrameID(), 50.0f);
        rosVisualizerInstance.visualizePointCloud("all_frontiers", every_frontier, explore_costmap_ros_->getLayeredCostmap()->getGlobalFrameID(), 500.0f);
        LOG_DEBUG("Frontiers visualized");
        EventLoggerInstance.endEvent("SearchForFrontiers", 0);
        explore_costmap_ros_->getCostmap()->getMutex()->unlock();
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {
        BT::OutputPort<std::vector<FrontierPtr>>("frontier_list"),
        BT::OutputPort<std::vector<std::vector<double>>>("every_frontier"),
        BT::InputPort<double>("increment_search_distance_by"),
        };
    }

    std::shared_ptr<FrontierSearchBase> frontierSearchPtr_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
    double original_search_distance_;
    double increment_value_;
    double current_search_distance_;
    };

    SearchForFrontiersPlugin::SearchForFrontiersPlugin()
    {
    }

    SearchForFrontiersPlugin::~SearchForFrontiersPlugin()
    {
    }

    void SearchForFrontiersPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<SearchForFrontiersBT>(
                name, 
                config,
                context->frontier_search,
                context->explore_costmap_ros,
                context->node);
        };
        factory.registerBuilder<SearchForFrontiersBT>("SearchForFrontiers", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::SearchForFrontiersPlugin,
    roadmap_explorer::BTPlugin)
