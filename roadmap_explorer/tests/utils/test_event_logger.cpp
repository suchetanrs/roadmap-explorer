#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

#include "roadmap_explorer/util/EventLogger.hpp"

class EventLoggerTest : public ::testing::Test 
{
protected:
    void SetUp() override 
    {
        // Ensure no existing instance
        try {
            EventLogger::destroyInstance();
        } catch (...) {
            // Instance may not exist, which is fine
        }
        
        // Create fresh instance for each test
        EventLogger::createInstance();
    }
    
    void TearDown() override 
    {
        // Clean up after each test
        try {
            EventLogger::destroyInstance();
        } catch (...) {
            // Instance may have been destroyed already
        }
    }
    
    // Helper function to sleep for a small amount
    void sleep_ms(int milliseconds) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }
    
    // Helper function to check if a duration is approximately correct
    bool isApproximatelyEqual(double actual, double expected, double tolerance = 0.05) 
    {
        return std::abs(actual - expected) <= tolerance;
    }
};

// Test singleton pattern functionality
TEST_F(EventLoggerTest, SingletonCreation) 
{
    // Should be able to get the instance after creating it in SetUp
    EXPECT_NO_THROW(EventLogger::getInstance());
    
    // Should throw if trying to create another instance
    EXPECT_THROW(EventLogger::createInstance(), RoadmapExplorerException);
}

TEST_F(EventLoggerTest, SingletonDestruction) 
{
    // Destroy the instance
    EventLogger::destroyInstance();
    
    // Should throw when trying to access destroyed instance
    EXPECT_THROW(EventLogger::getInstance(), RoadmapExplorerException);
    
    // Should be able to create a new instance after destruction
    EXPECT_NO_THROW(EventLogger::createInstance());
    EXPECT_NO_THROW(EventLogger::getInstance());
}

TEST_F(EventLoggerTest, MultipleDestroyCallsSafe) 
{
    // First destroy should work
    EXPECT_NO_THROW(EventLogger::destroyInstance());
    
    // Second destroy should not crash
    EXPECT_NO_THROW(EventLogger::destroyInstance());
    
    // Should still throw when trying to access
    EXPECT_THROW(EventLogger::getInstance(), RoadmapExplorerException);
}

// Test basic event timing functionality
TEST_F(EventLoggerTest, BasicEventTiming) 
{
    auto& logger = EventLogger::getInstance();
    
    const std::string event_name = "test_event";
    
    // Start an event
    EXPECT_NO_THROW(logger.startEvent(event_name));
    
    // Sleep for a known duration
    sleep_ms(100);
    
    // End the event
    EXPECT_NO_THROW(logger.endEvent(event_name, 2));
}

TEST_F(EventLoggerTest, EventTimingAccuracy) 
{
    auto& logger = EventLogger::getInstance();
    
    const std::string event_name = "timing_accuracy_test";
    
    logger.startEvent(event_name);
    sleep_ms(200);  // Sleep for 200ms
    
    // Check time since start
    double elapsed = logger.getTimeSinceStart(event_name);
    EXPECT_TRUE(isApproximatelyEqual(elapsed, 0.2, 0.1));  // Should be around 0.2 seconds
    
    // Sleep a bit more
    sleep_ms(100);  // Additional 100ms
    elapsed = logger.getTimeSinceStart(event_name);
    EXPECT_TRUE(isApproximatelyEqual(elapsed, 0.3, 0.1));  // Should be around 0.3 seconds
    
    logger.endEvent(event_name, 2);
}

// Test different event levels
TEST_F(EventLoggerTest, DifferentEventLevels) 
{
    auto& logger = EventLogger::getInstance();
    
    // Test all valid event levels
    std::vector<int> valid_levels = {-1, 0, 1, 2};
    
    for (int level : valid_levels) {
        std::string event_name = "level_" + std::to_string(level) + "_event";
        
        logger.startEvent(event_name);
        sleep_ms(10);
        EXPECT_NO_THROW(logger.endEvent(event_name, level));
    }
}

TEST_F(EventLoggerTest, InvalidEventLevel) 
{
    auto& logger = EventLogger::getInstance();
    
    const std::string event_name = "invalid_level_event";
    
    logger.startEvent(event_name);
    sleep_ms(10);
    
    // Invalid event level should not crash, but should log a critical message
    EXPECT_NO_THROW(logger.endEvent(event_name, 999));
}

// Test multiple concurrent events
TEST_F(EventLoggerTest, MultipleConcurrentEvents) 
{
    auto& logger = EventLogger::getInstance();
    
    // Start multiple events
    logger.startEvent("event1");
    sleep_ms(50);
    
    logger.startEvent("event2");
    sleep_ms(50);
    
    logger.startEvent("event3");
    sleep_ms(50);
    
    // Check timing for each event
    double elapsed1 = logger.getTimeSinceStart("event1");
    double elapsed2 = logger.getTimeSinceStart("event2");
    double elapsed3 = logger.getTimeSinceStart("event3");
    
    // event1 should have run longest
    EXPECT_GT(elapsed1, elapsed2);
    EXPECT_GT(elapsed2, elapsed3);
    
    // End events
    logger.endEvent("event3", 2);
    logger.endEvent("event2", 1);
    logger.endEvent("event1", 0);
}

// Test error cases
TEST_F(EventLoggerTest, EndEventWithoutStart) 
{
    auto& logger = EventLogger::getInstance();
    
    // Try to end an event that was never started
    EXPECT_NO_THROW(logger.endEvent("nonexistent_event", 2));
}

TEST_F(EventLoggerTest, GetTimeWithoutStart) 
{
    auto& logger = EventLogger::getInstance();
    
    // Try to get time for an event that was never started
    double elapsed = logger.getTimeSinceStart("nonexistent_event");
    EXPECT_EQ(elapsed, 0.0);
}

TEST_F(EventLoggerTest, DoubleEndEvent) 
{
    auto& logger = EventLogger::getInstance();
    
    const std::string event_name = "double_end_event";
    
    logger.startEvent(event_name);
    sleep_ms(10);
    
    // First end should work
    EXPECT_NO_THROW(logger.endEvent(event_name, 2));
    
    // Second end should not crash (event should be removed from map)
    EXPECT_NO_THROW(logger.endEvent(event_name, 2));
}

TEST_F(EventLoggerTest, RestartSameEvent) 
{
    auto& logger = EventLogger::getInstance();
    
    const std::string event_name = "restart_event";
    
    // First cycle
    logger.startEvent(event_name);
    sleep_ms(50);
    logger.endEvent(event_name, 2);
    
    // Second cycle - should be able to reuse the same event name
    logger.startEvent(event_name);
    sleep_ms(30);
    double elapsed = logger.getTimeSinceStart(event_name);
    EXPECT_TRUE(isApproximatelyEqual(elapsed, 0.03, 0.02));
    
    logger.endEvent(event_name, 2);
}

// Test planning count functionality
TEST_F(EventLoggerTest, PlanningCountIncrement) 
{
    auto& logger = EventLogger::getInstance();
    
    // Increment planning count multiple times
    EXPECT_NO_THROW(logger.incrementPlanningCount());
    EXPECT_NO_THROW(logger.incrementPlanningCount());
    EXPECT_NO_THROW(logger.incrementPlanningCount());
    
    // Should not crash or throw exceptions
}

// Test edge cases with event names
TEST_F(EventLoggerTest, EmptyEventName) 
{
    auto& logger = EventLogger::getInstance();
    
    // Empty string as event name
    logger.startEvent("");
    sleep_ms(10);
    
    double elapsed = logger.getTimeSinceStart("");
    EXPECT_GT(elapsed, 0.0);
    
    EXPECT_NO_THROW(logger.endEvent("", 2));
}

TEST_F(EventLoggerTest, LongEventName) 
{
    auto& logger = EventLogger::getInstance();
    
    // Very long event name
    std::string long_name(1000, 'a');
    
    logger.startEvent(long_name);
    sleep_ms(10);
    
    double elapsed = logger.getTimeSinceStart(long_name);
    EXPECT_GT(elapsed, 0.0);
    
    EXPECT_NO_THROW(logger.endEvent(long_name, 2));
}

TEST_F(EventLoggerTest, SpecialCharacterEventName) 
{
    auto& logger = EventLogger::getInstance();
    
    // Event name with special characters
    std::string special_name = "event!@#$%^&*()_+-=[]{}|;':\",./<>?`~";
    
    logger.startEvent(special_name);
    sleep_ms(10);
    
    double elapsed = logger.getTimeSinceStart(special_name);
    EXPECT_GT(elapsed, 0.0);
    
    EXPECT_NO_THROW(logger.endEvent(special_name, 2));
}

// Test thread safety (basic test)
TEST_F(EventLoggerTest, BasicThreadSafety) 
{
    auto& logger = EventLogger::getInstance();
    
    // Start multiple events from different threads
    std::vector<std::thread> threads;
    
    for (int i = 0; i < 5; ++i) {
        threads.emplace_back([&logger, i]() {
            std::string event_name = "thread_event_" + std::to_string(i);
            logger.startEvent(event_name);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            logger.endEvent(event_name, 2);
        });
    }
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
    
    // If we reach here without crashing, basic thread safety is working
    SUCCEED();
}

// Test profiler functionality
TEST_F(EventLoggerTest, ProfilerBasicUsage) 
{
    {
        // Create a profiler scope
        Profiler profiler("test_function");
        sleep_ms(50);
        // Profiler should automatically end when it goes out of scope
    }
    
    // If we reach here without crashing, profiler worked
    SUCCEED();
}

TEST_F(EventLoggerTest, ProfilerNesting) 
{
    {
        Profiler outer_profiler("outer_function");
        sleep_ms(25);
        
        {
            Profiler inner_profiler("inner_function");
            sleep_ms(25);
            // Inner profiler ends here
        }
        
        sleep_ms(25);
        // Outer profiler ends here
    }
    
    SUCCEED();
}

// Test macro usage
TEST_F(EventLoggerTest, ProfileMacroUsage) 
{
    auto test_function = []() {
        PROFILE_FUNCTION;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };
    
    EXPECT_NO_THROW(test_function());
}

// Test high frequency operations
TEST_F(EventLoggerTest, HighFrequencyOperations) 
{
    auto& logger = EventLogger::getInstance();
    
    // Perform many rapid operations
    for (int i = 0; i < 100; ++i) {
        std::string event_name = "rapid_event_" + std::to_string(i);
        logger.startEvent(event_name);
        // Very brief wait
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        logger.endEvent(event_name, 2);
    }
    
    SUCCEED();
}

// Test zero duration events
TEST_F(EventLoggerTest, ZeroDurationEvents) 
{
    auto& logger = EventLogger::getInstance();
    
    const std::string event_name = "zero_duration_event";
    
    logger.startEvent(event_name);
    // End immediately without sleeping
    logger.endEvent(event_name, 2);
    
    SUCCEED();
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
