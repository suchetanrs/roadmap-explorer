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

#include "roadmap_explorer/bt_plugins/UpdateBoundaryPlugin.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace roadmap_explorer
{
    class UpdateBoundaryPolygonBT : public BT::SyncActionNode
    {
    public:
        UpdateBoundaryPolygonBT(
            const std::string &name, 
            const BT::NodeConfiguration &config,
            std::shared_ptr<CostAssigner> cost_assigner_ptr,
            std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
            : BT::SyncActionNode(name, config)
        {
            cost_assigner_ptr_ = cost_assigner_ptr;
            config_ = parameterInstance.getValue<std::vector<double>>("explorationBT.exploration_boundary");
            ros_node_ptr_ = ros_node_ptr;
            LOG_DEBUG("UpdateBoundaryPolygonBT Constructor");
        }

        BT::NodeStatus tick() override
        {
            LOG_FLOW("MODULE UpdateBoundaryPolygonBT");
            geometry_msgs::msg::PolygonStamped explore_boundary_;
            explore_boundary_.header.frame_id = "map";
            explore_boundary_.header.stamp = rclcpp::Clock().now();
            for (int i = 0; i < (int)config_.size(); i += 2) {
            geometry_msgs::msg::Point32 point;
            point.x = config_[i];
            point.y = config_[i + 1];
            LOG_DEBUG("Adding point to boundary: " << point.x << ", " << point.y);
            explore_boundary_.polygon.points.push_back(point);
            }

            cost_assigner_ptr_->updateBoundaryPolygon(explore_boundary_);
            return BT::NodeStatus::SUCCESS;
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
        std::shared_ptr<CostAssigner> cost_assigner_ptr_;
        std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
        std::vector<double> config_;
    };

    

    UpdateBoundaryPlugin::UpdateBoundaryPlugin()
    {
    }

    UpdateBoundaryPlugin::~UpdateBoundaryPlugin()
    {
    }

    void UpdateBoundaryPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
    {
        BT::NodeBuilder builder_update_boundary =
            [context](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<UpdateBoundaryPolygonBT>(
                name, 
                config, 
                context->cost_assigner,
                context->node);
        };
        factory.registerBuilder<UpdateBoundaryPolygonBT>("UpdateBoundaryPolygon", builder_update_boundary);
    }
}

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::UpdateBoundaryPlugin,
    roadmap_explorer::BTPlugin)
