#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Frontier.hpp"

int main() {
    FrontierPtr frontier = std::make_shared<Frontier>();
    frontier->setGoalPoint(9.0, 98.0);
    frontier->setPathLength(5.0);
    frontier->setPathLengthInM(66);
    // Test the logging macros
    LOG_TRACE("This is a TRACE message." << 5);
    LOG_DEBUG("This is a DEBUG message.\n" << frontier);
    LOG_INFO("This is an INFO message.");
    LOG_WARN("This is a WARN message.");
    LOG_ERROR("This is an ERROR message.");
    LOG_CRITICAL("This is a CRITICAL message.");
    LOG_FATAL("This is a FATAL message.");
    LOG_HIGHLIGHT("This is a HIGHLIGHT message.");
    LOG_FLOW("This is a FLOW message");

    LOG_MODULE_TIME("This is LOG_MODULE_TIME message", 5);
    LOG_SUBMODULE_TIME("This is LOG_SUBMODULE_TIME message", 5);
    LOG_EVENT_TIME("This is LOG_EVENT_TIME message", 5);

    return 0;
}