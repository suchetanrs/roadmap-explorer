// core_plugins.hpp
#pragma once

#include <roadmap_explorer/bt_plugins/BaseBTPlugin.hpp>
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"

#include "roadmap_explorer/Nav2Interface.hpp"

namespace roadmap_explorer
{
    class SendNav2GoalPlugin : public BTPlugin
    {
        public:
        SendNav2GoalPlugin();

        ~SendNav2GoalPlugin();

        void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<BTContext> context) override;
    };
};
