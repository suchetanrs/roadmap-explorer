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

#include "roadmap_explorer/bt_plugins/SendNav2GoalPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class SendNav2Goal : public BT::StatefulActionNode
    {
    public:
    SendNav2Goal(
        const std::string & name, const BT::NodeConfiguration & config,
        std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface,
        std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
    : BT::StatefulActionNode(name, config)
    {
        nav2_interface_ = nav2_interface;
        ros_node_ptr_ = ros_node_ptr;
        LOG_INFO("SendNav2Goal Constructor");
    }

    BT::NodeStatus onStart() override
    {
        LOG_FLOW("SendNav2Goal onStart");
        FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
        getInput("allocated_frontier", allocatedFrontier);
        geometry_msgs::msg::PoseStamped goalPose;
        goalPose.header.frame_id = "map";
        goalPose.pose.position = allocatedFrontier->getGoalPoint();
        goalPose.pose.orientation = allocatedFrontier->getGoalOrientation();
        if (!nav2_interface_->canSendNewGoal()) {
        LOG_ERROR(
            "Nav2Interface cannot send new goal, goal is already active. Status:" <<
            nav2_interface_->getGoalStatus());
        // dont return failure if it's ongoing. It's probably on going from the previous iteration.
        if (nav2_interface_->getGoalStatus() != NavGoalStatus::ONGOING) {
            config().blackboard->set<ExplorationErrorCode>(
            "error_code_id", ExplorationErrorCode::UNHANDLED_ERROR);
            return BT::NodeStatus::FAILURE;
        }
        }
        else
        {
        nav2_interface_->sendGoal(goalPose);
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()
    {
        LOG_DEBUG("SendNav2Goal onRunning");
        FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
        getInput("allocated_frontier", allocatedFrontier);
        LOG_DEBUG("Sending goal " << allocatedFrontier);
        if (nav2_interface_->getGoalStatus() == NavGoalStatus::SENDING_GOAL) {
        LOG_INFO("Nav2 goal is being sent, waiting for response...");
        return BT::NodeStatus::RUNNING;
        }
        if (nav2_interface_->getGoalStatus() == NavGoalStatus::ONGOING) {
        LOG_INFO("Nav2 goal is ongoing, waiting for completion...");
        geometry_msgs::msg::PoseStamped goalPose;
        goalPose.header.frame_id = "map";
        goalPose.pose.position = allocatedFrontier->getGoalPoint();
        goalPose.pose.orientation = allocatedFrontier->getGoalOrientation();
        LOG_TRACE(
            "Current goal pose: " << goalPose.pose.position.x << ", " << goalPose.pose.position.y << ", " << goalPose.pose.orientation.z << ", " << goalPose.pose.orientation.w << ", " << goalPose.pose.orientation.x << ", " <<
            goalPose.pose.orientation.y);
        nav2_interface_->sendUpdatedGoal(goalPose);
        return BT::NodeStatus::RUNNING;
        }
        if (nav2_interface_->getGoalStatus() == NavGoalStatus::FAILED) {
        LOG_ERROR("Nav2 goal has aborted!");
        config().blackboard->set<ExplorationErrorCode>(
            "error_code_id", ExplorationErrorCode::NAV2_GOAL_ABORT);
        config().blackboard->set<FrontierPtr>(
            "latest_failed_frontier", allocatedFrontier);
        return BT::NodeStatus::FAILURE;
        }
        if (nav2_interface_->getGoalStatus() == NavGoalStatus::SUCCEEDED) {
        LOG_WARN("Nav2 goal has succeeded!");
        }
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted()
    {
        LOG_INFO("SendNav2Goal onHalted");
        return;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<FrontierPtr>("allocated_frontier")};
    }

    std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface_;
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
    };

    SendNav2GoalPlugin::SendNav2GoalPlugin()
    {
    }

    SendNav2GoalPlugin::~SendNav2GoalPlugin()
    {
    }

    void SendNav2GoalPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<SendNav2Goal>(
                name, 
                config,
                context->nav2_interface,
                context->node);
        };
        factory.registerBuilder<SendNav2Goal>("SendNav2Goal", builder);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::SendNav2GoalPlugin,
    roadmap_explorer::BTPlugin)
