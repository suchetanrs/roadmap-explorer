// update_boundary_plugin.hpp
#pragma once

#include <roadmap_explorer/bt_plugins/BaseBTPlugin.hpp>
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"

#include "roadmap_explorer/planners/FrontierRoadmap.hpp"
#include "roadmap_explorer/FullPathOptimizer.hpp"

namespace roadmap_explorer
{
    class UpdateRoadmapPlugin : public BTPlugin
    {
        public:
        UpdateRoadmapPlugin();

        ~UpdateRoadmapPlugin();

        void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<BTContext> context) override;
    };
};