// update_boundary_plugin.hpp
#pragma once

#include <roadmap_explorer/bt_plugins/BaseBTPlugin.hpp>
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"

namespace roadmap_explorer
{
    class UpdateBoundaryPlugin : public BTPlugin
    {
        public:
        UpdateBoundaryPlugin();

        ~UpdateBoundaryPlugin();

        void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<BTContext> context) override;
    };
};