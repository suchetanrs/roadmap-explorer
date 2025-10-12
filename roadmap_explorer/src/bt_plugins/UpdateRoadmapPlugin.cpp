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

#include "roadmap_explorer/bt_plugins/UpdateRoadmapPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class UpdateRoadmapBT : public BT::SyncActionNode
    {
    public:
    UpdateRoadmapBT(
        const std::string & name, const BT::NodeConfiguration & config,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
    : BT::SyncActionNode(name, config)
    {
        explore_costmap_ros_ = explore_costmap_ros;
        LOG_INFO("UpdateRoadmapBT Constructor");
    }

    BT::NodeStatus tick() override
    {
        EventLoggerInstance.incrementPlanningCount();
        EventLoggerInstance.startEvent("UpdateRoadmapBT");
        LOG_FLOW("MODULE UpdateRoadmapBT");
        std::vector<FrontierPtr> frontier_list;
        getInput<std::vector<FrontierPtr>>("frontier_list", frontier_list);
        LOG_INFO("Recieved " << frontier_list.size() << " frontiers in UpdateRoadmapBT");
        geometry_msgs::msg::PoseStamped robotP;
        if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP)) {
        // Handle the case when "latest_robot_pose" is not found
        LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
        throw RoadmapExplorerException("Failed to retrieve latest_robot_pose from blackboard.");
        }
        frontierRoadmapInstance.addNodes(frontier_list, true);
        bool addPose;
        getInput("add_robot_pose_to_roadmap", addPose);
        if (addPose) {
        LOG_FLOW("Adding robot pose as frontier node.");
        frontierRoadmapInstance.addRobotPoseAsNode(robotP.pose, true);
        }
        EventLoggerInstance.startEvent("roadmapReconstruction");
        frontierRoadmapInstance.constructNewEdges(frontier_list);
        frontierRoadmapInstance.constructNewEdgeRobotPose(robotP.pose);
        // frontierRoadmapInstance.reConstructGraph();
        EventLoggerInstance.endEvent("roadmapReconstruction", 1);

        EventLoggerInstance.startEvent("publishRoadmap");
        frontierRoadmapInstance.publishRoadMap();
        EventLoggerInstance.endEvent("publishRoadmap", 2);
        // TODO: make sure to add a thing such that the entire roadmap within a certain distance (max frontier search distance) is reconstructed periodically
        EventLoggerInstance.endEvent("UpdateRoadmapBT", 0);
        // frontierRoadmapInstance.countTotalItemsInSpatialMap();
        // TODO: remove below line
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {
        BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
        BT::InputPort<bool>("add_robot_pose_to_roadmap")};
    }

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    };

    

    UpdateRoadmapPlugin::UpdateRoadmapPlugin()
    {
    }

    UpdateRoadmapPlugin::~UpdateRoadmapPlugin()
    {
    }

    void UpdateRoadmapPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<UpdateRoadmapBT>(
                name, 
                config, 
                context->explore_costmap_ros);
        };
        factory.registerBuilder<UpdateRoadmapBT>("UpdateFrontierRoadmap", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::UpdateRoadmapPlugin,
    roadmap_explorer::BTPlugin)
