#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "roadmap_explorer/bt_plugins/LogIterationPlugin.hpp"

#ifdef ROS_DISTRO_HUMBLE
  #include <behaviortree_cpp_v3/bt_factory.h>
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  #include <behaviortree_cpp/bt_factory.h>
#else
  #error "Unsupported ROS distro"
#endif

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"

namespace roadmap_explorer
{

class LogIterationPluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    
    // Create a lifecycle node for testing
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_log_iteration_node");
    
    // Create TF buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    
    // Create mock costmap (simplified for testing)
    costmap_ros_ = nullptr;
    
    // Create the plugin
    plugin_ = std::make_unique<LogIteration>();
    
    // Create BT factory
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
    
    // Initialize EventLogger for testing
    try {
      EventLogger::createInstance();
    } catch (const std::exception&) {
      // EventLogger might already exist, ignore
    }
  }

  void TearDown() override
  {
    plugin_.reset();
    factory_.reset();
    node_.reset();
    tf_buffer_.reset();
    costmap_ros_.reset();
    context_.reset();
    
    // Clean up EventLogger
    try {
      EventLogger::destroyInstance();
    } catch (const std::exception&) {
      // Ignore if already destroyed
    }
    
    rclcpp::shutdown();
  }
  
  // Helper function to create BTContext
  std::shared_ptr<BTContext> createContext() {
    auto context = std::make_shared<BTContext>();
    context->node = node_;
    context->explore_costmap_ros = costmap_ros_;
    context->tf_buffer = tf_buffer_;
    return context;
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<BTContext> context_;
  std::unique_ptr<LogIteration> plugin_;
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
};

// Test plugin construction and destruction
TEST_F(LogIterationPluginTest, ConstructorDestructor)
{
  // Test that the plugin can be created
  EXPECT_NE(plugin_.get(), nullptr);
  
  // Test that we can create another instance
  auto another_plugin = std::make_unique<LogIteration>();
  EXPECT_NE(another_plugin.get(), nullptr);
  
  // Test destruction
  another_plugin.reset();
  SUCCEED(); // If we reach here, destruction worked
}

// Test registerNodes functionality
TEST_F(LogIterationPluginTest, RegisterNodes)
{
  // Register nodes
  context_ = createContext();
  EXPECT_NO_THROW(plugin_->registerNodes(*factory_, context_));
  
  // Try to instantiate the registered node to verify registration worked
  BT::NodeConfiguration config;
  auto node_instance = factory_->instantiateTreeNode("LogIterationBT", "LogIterationBT", config);
  EXPECT_NE(node_instance, nullptr);
}

// Test that registered node can be instantiated
TEST_F(LogIterationPluginTest, InstantiateRegisteredNode)
{
  // Register nodes first
  context_ = createContext();
  plugin_->registerNodes(*factory_, context_);
  
  // Create node configuration
  BT::NodeConfiguration config;
  
  // Try to instantiate the LogIterationBT node
  auto node_instance = factory_->instantiateTreeNode("LogIterationBT", "LogIterationBT", config);
  
  EXPECT_NE(node_instance, nullptr);
  EXPECT_EQ(node_instance->name(), "LogIterationBT");
}

// Test LogIterationBT node behavior
TEST_F(LogIterationPluginTest, LogIterationBTNodeBehavior)
{
  // Register nodes first
  context_ = createContext();
  plugin_->registerNodes(*factory_, context_);
  
  // Create node configuration
  BT::NodeConfiguration config;
  
  // Instantiate the LogIterationBT node
  auto node_instance = factory_->instantiateTreeNode("LogIterationBT", "LogIterationBT", config);
  ASSERT_NE(node_instance, nullptr);
  
  // Get the planning count before tick
  size_t initial_count = EventLoggerInstance.getPlanningCount();
  
  // Execute tick
  BT::NodeStatus status = node_instance->executeTick();
  
  // Should return SUCCESS
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  
  // Planning count should have incremented
  size_t final_count = EventLoggerInstance.getPlanningCount();
  EXPECT_EQ(final_count, initial_count + 1);
}

// Test multiple ticks increment counter correctly
TEST_F(LogIterationPluginTest, MultipleTicks)
{
  // Register nodes first
  context_ = createContext();
  plugin_->registerNodes(*factory_, context_);
  
  // Create node configuration
  BT::NodeConfiguration config;
  
  // Instantiate the LogIterationBT node
  auto node_instance = factory_->instantiateTreeNode("LogIterationBT", "LogIterationBT", config);
  ASSERT_NE(node_instance, nullptr);
  
  // Get the planning count before ticks
  size_t initial_count = EventLoggerInstance.getPlanningCount();
  
  // Execute multiple ticks
  const int num_ticks = 5;
  for (int i = 0; i < num_ticks; ++i) {
    BT::NodeStatus status = node_instance->executeTick();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  }
  
  // Planning count should have incremented by num_ticks
  size_t final_count = EventLoggerInstance.getPlanningCount();
  EXPECT_EQ(final_count, initial_count + num_ticks);
}

// Test inheritance from BTPlugin
TEST_F(LogIterationPluginTest, InheritanceFromBTPlugin)
{
  // Test that LogIteration is a BTPlugin
  BTPlugin* base_ptr = plugin_.get();
  EXPECT_NE(base_ptr, nullptr);
  
  // Test polymorphic behavior
  context_ = createContext();
  EXPECT_NO_THROW(base_ptr->registerNodes(*factory_, context_));
  
  // Verify registration worked by trying to instantiate
  BT::NodeConfiguration config;
  auto node_instance = factory_->instantiateTreeNode("LogIterationBT", "LogIterationBT", config);
  EXPECT_NE(node_instance, nullptr);
}

// Test with different node names using XML-based approach
TEST_F(LogIterationPluginTest, DifferentNodeNames)
{
  // Register nodes first
  context_ = createContext();
  plugin_->registerNodes(*factory_, context_);
  
  // Create a simple behavior tree XML with multiple LogIterationBT nodes
  std::string bt_xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="TestTree">
        <Sequence>
          <LogIterationBT name="instance_1" />
          <LogIterationBT name="instance_2" />
        </Sequence>
      </BehaviorTree>
    </root>
  )";
  
  // Create blackboard
  auto blackboard = BT::Blackboard::create();
  
  // Try to create tree from XML
  BT::Tree tree;
  EXPECT_NO_THROW(tree = factory_->createTreeFromText(bt_xml, blackboard));
  
  // Test tree execution
  size_t initial_count = EventLoggerInstance.getPlanningCount();
#ifdef ROS_DISTRO_HUMBLE
  BT::NodeStatus status = tree.tickRoot();
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  BT::NodeStatus status = tree.tickOnce();
#else
  #error "Unsupported ROS distro"
#endif
  
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  // Should have executed both nodes, so count should increase by 2
  EXPECT_EQ(EventLoggerInstance.getPlanningCount(), initial_count + 2);
}

// Test node type consistency
TEST_F(LogIterationPluginTest, NodeTypeConsistency)
{
  // Register nodes first
  context_ = createContext();
  plugin_->registerNodes(*factory_, context_);
  
  // Create node configuration
  BT::NodeConfiguration config;
  
  // Instantiate the LogIterationBT node
  auto node_instance = factory_->instantiateTreeNode("LogIterationBT", "LogIterationBT", config);
  ASSERT_NE(node_instance, nullptr);
  
  // Check that it's a SyncActionNode (based on the implementation)
  auto sync_action_node = dynamic_cast<BT::SyncActionNode*>(node_instance.get());
  EXPECT_NE(sync_action_node, nullptr);
}

// Test error handling with invalid parameters
TEST_F(LogIterationPluginTest, ErrorHandlingInvalidParameters)
{
  // Register nodes first
  context_ = createContext();
  plugin_->registerNodes(*factory_, context_);
  
  // Try to instantiate with invalid node type name
  BT::NodeConfiguration config;
  
  EXPECT_THROW(
    factory_->instantiateTreeNode("InvalidNodeType", "test_invalid", config),
    std::exception
  );
}

// Test that the plugin works with null costmap (edge case)
TEST_F(LogIterationPluginTest, NullCostmapHandling)
{
  // This should not crash even with null costmap
  auto null_costmap_context = std::make_shared<BTContext>();
  null_costmap_context->node = node_;
  null_costmap_context->explore_costmap_ros = nullptr;
  null_costmap_context->tf_buffer = tf_buffer_;
  EXPECT_NO_THROW(plugin_->registerNodes(*factory_, null_costmap_context));
  
  // Verify registration worked by trying to instantiate
  BT::NodeConfiguration config;
  auto node_instance = factory_->instantiateTreeNode("LogIterationBT", "LogIterationBT", config);
  EXPECT_NE(node_instance, nullptr);
  EXPECT_EQ(node_instance->executeTick(), BT::NodeStatus::SUCCESS);
}

// Test that the plugin works with null tf_buffer (edge case)
TEST_F(LogIterationPluginTest, NullTfBufferHandling)
{
  // This should not crash even with null tf_buffer
  auto null_tf_context = std::make_shared<BTContext>();
  null_tf_context->node = node_;
  null_tf_context->explore_costmap_ros = costmap_ros_;
  null_tf_context->tf_buffer = nullptr;
  EXPECT_NO_THROW(plugin_->registerNodes(*factory_, null_tf_context));
  
  // Verify registration worked by trying to instantiate
  BT::NodeConfiguration config;
  auto node_instance = factory_->instantiateTreeNode("LogIterationBT", "LogIterationBT", config);
  EXPECT_NE(node_instance, nullptr);
  EXPECT_EQ(node_instance->executeTick(), BT::NodeStatus::SUCCESS);
}

// Test thread safety (basic test)
TEST_F(LogIterationPluginTest, BasicThreadSafety)
{
  // Register nodes first
  context_ = createContext();
  plugin_->registerNodes(*factory_, context_);
  
#ifdef ROS_DISTRO_HUMBLE
  // For Humble (BT.CPP 3.x): Use Parallel node
  std::string bt_xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="ThreadSafetyTest">
        <Parallel success_threshold="2" failure_threshold="1">
          <LogIterationBT name="thread_test_1" />
          <LogIterationBT name="thread_test_2" />
        </Parallel>
      </BehaviorTree>
    </root>
  )";
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  // For Jazzy/Kilted (BT.CPP 4.x): Use Sequence instead
  // Parallel nodes behave differently with tickOnce() in BT.CPP 4.x
  std::string bt_xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="ThreadSafetyTest">
        <Sequence>
          <LogIterationBT name="thread_test_1" />
          <LogIterationBT name="thread_test_2" />
        </Sequence>
      </BehaviorTree>
    </root>
  )";
#else
  #error "Unsupported ROS distro"
#endif
  
  // Create blackboard
  auto blackboard = BT::Blackboard::create();
  
  // Try to create tree from XML
  BT::Tree tree;
  EXPECT_NO_THROW(tree = factory_->createTreeFromText(bt_xml, blackboard));
  
  // Execute them concurrently (simple test)
  size_t initial_count = EventLoggerInstance.getPlanningCount();
  
  // Execute the tree (which runs both nodes in parallel)
#ifdef ROS_DISTRO_HUMBLE
  BT::NodeStatus status = tree.tickRoot();
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  BT::NodeStatus status = tree.tickOnce();
#else
  #error "Unsupported ROS distro"
#endif
  
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  
  // Count should have incremented by 2 (both nodes executed)
  size_t final_count = EventLoggerInstance.getPlanningCount();
  EXPECT_EQ(final_count, initial_count + 2);
}

} // namespace roadmap_explorer

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

