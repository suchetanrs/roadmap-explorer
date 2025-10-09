#include <gtest/gtest.h>
#include <memory>

#include "roadmap_explorer/bt_plugins/BaseBTPlugin.hpp"
#include "roadmap_explorer/ExplorationBT.hpp"

#ifdef ROS_DISTRO_HUMBLE
  #include <behaviortree_cpp_v3/bt_factory.h>
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  #include <behaviortree_cpp/bt_factory.h>
#else
  #error "Unsupported ROS distro"
#endif

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace roadmap_explorer
{

// Mock implementation of BTPlugin for testing
class MockBTPlugin : public BTPlugin
{
public:
  MockBTPlugin() : register_nodes_called_(false) {}
  
  void registerNodes(
    BT::BehaviorTreeFactory & factory,
    std::shared_ptr<nav2_util::LifecycleNode> node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer) override
  {
    register_nodes_called_ = true;
    factory_ = &factory;
    node_ = node;
    costmap_ros_ = explore_costmap_ros;
    tf_buffer_ = tf_buffer;
  }
  
  bool wasRegisterNodesCalled() const { return register_nodes_called_; }
  BT::BehaviorTreeFactory* getFactory() const { return factory_; }
  std::shared_ptr<nav2_util::LifecycleNode> getNode() const { return node_; }
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getCostmapRos() const { return costmap_ros_; }
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const { return tf_buffer_; }

private:
  bool register_nodes_called_;
  BT::BehaviorTreeFactory* factory_;
  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

class BaseBTPluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    
    // Create a lifecycle node for testing
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");
    
    // Create TF buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    
    // Create mock costmap (note: this is a simplified setup for testing)
    // In a real scenario, you might need a more complete costmap setup
    costmap_ros_ = nullptr; // We'll test with nullptr for now
    
    // Create the mock plugin
    mock_plugin_ = std::make_unique<MockBTPlugin>();
    
    // Create BT factory
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
  }

  void TearDown() override
  {
    mock_plugin_.reset();
    factory_.reset();
    node_.reset();
    tf_buffer_.reset();
    costmap_ros_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<MockBTPlugin> mock_plugin_;
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
};

// Test ExplorationErrorCode enum values
TEST_F(BaseBTPluginTest, ExplorationErrorCodeValues)
{
  // Test that all error codes have expected values
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::NO_FRONTIERS_IN_CURRENT_RADIUS), 0);
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::MAX_FRONTIER_SEARCH_RADIUS_EXCEEDED), 1);
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::COST_COMPUTATION_FAILURE), 2);
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::NO_ACHIEVABLE_FRONTIERS_LEFT), 3);
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::FULL_PATH_OPTIMIZATION_FAILURE), 4);
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::REFINED_PATH_COMPUTATION_FAILURE), 5);
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::UNHANDLED_ERROR), 6);
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::NAV2_GOAL_ABORT), 7);
  EXPECT_EQ(static_cast<int>(ExplorationErrorCode::NO_ERROR), 8);
}

// Test BTPlugin interface
TEST_F(BaseBTPluginTest, BTPluginInterface)
{
  // Test that the plugin can be created
  EXPECT_NE(mock_plugin_.get(), nullptr);
  
  // Test that registerNodes hasn't been called yet
  EXPECT_FALSE(mock_plugin_->wasRegisterNodesCalled());
  
  // Call registerNodes
  mock_plugin_->registerNodes(*factory_, node_, costmap_ros_, tf_buffer_);
  
  // Verify that registerNodes was called
  EXPECT_TRUE(mock_plugin_->wasRegisterNodesCalled());
  
  // Verify that the parameters were stored correctly
  EXPECT_EQ(mock_plugin_->getFactory(), factory_.get());
  EXPECT_EQ(mock_plugin_->getNode(), node_);
  EXPECT_EQ(mock_plugin_->getCostmapRos(), costmap_ros_);
  EXPECT_EQ(mock_plugin_->getTfBuffer(), tf_buffer_);
}

// Test virtual destructor
TEST_F(BaseBTPluginTest, VirtualDestructor)
{
  // Create a plugin through base pointer
  std::unique_ptr<BTPlugin> plugin = std::make_unique<MockBTPlugin>();
  
  // This should not cause any issues due to virtual destructor
  plugin.reset();
  
  // If we reach here, the virtual destructor worked correctly
  SUCCEED();
}

// Test polymorphic behavior
TEST_F(BaseBTPluginTest, PolymorphicBehavior)
{
  // Create plugin through base pointer
  std::unique_ptr<BTPlugin> plugin = std::make_unique<MockBTPlugin>();
  
  // Cast back to derived type to check state
  MockBTPlugin* mock_ptr = dynamic_cast<MockBTPlugin*>(plugin.get());
  ASSERT_NE(mock_ptr, nullptr);
  
  // Test that registerNodes can be called through base pointer
  EXPECT_FALSE(mock_ptr->wasRegisterNodesCalled());
  
  plugin->registerNodes(*factory_, node_, costmap_ros_, tf_buffer_);
  
  EXPECT_TRUE(mock_ptr->wasRegisterNodesCalled());
}

// Test error code comparison and assignment
TEST_F(BaseBTPluginTest, ExplorationErrorCodeComparison)
{
  ExplorationErrorCode error1 = ExplorationErrorCode::NO_ERROR;
  ExplorationErrorCode error2 = ExplorationErrorCode::NO_ERROR;
  ExplorationErrorCode error3 = ExplorationErrorCode::UNHANDLED_ERROR;
  
  // Test equality
  EXPECT_EQ(error1, error2);
  EXPECT_NE(error1, error3);
  
  // Test assignment
  error1 = ExplorationErrorCode::COST_COMPUTATION_FAILURE;
  EXPECT_EQ(error1, ExplorationErrorCode::COST_COMPUTATION_FAILURE);
  EXPECT_NE(error1, error2);
}

// Test that all error codes are distinct
TEST_F(BaseBTPluginTest, ExplorationErrorCodeDistinct)
{
  std::vector<ExplorationErrorCode> all_codes = {
    ExplorationErrorCode::NO_FRONTIERS_IN_CURRENT_RADIUS,
    ExplorationErrorCode::MAX_FRONTIER_SEARCH_RADIUS_EXCEEDED,
    ExplorationErrorCode::COST_COMPUTATION_FAILURE,
    ExplorationErrorCode::NO_ACHIEVABLE_FRONTIERS_LEFT,
    ExplorationErrorCode::FULL_PATH_OPTIMIZATION_FAILURE,
    ExplorationErrorCode::REFINED_PATH_COMPUTATION_FAILURE,
    ExplorationErrorCode::UNHANDLED_ERROR,
    ExplorationErrorCode::NAV2_GOAL_ABORT,
    ExplorationErrorCode::NO_ERROR
  };
  
  // Check that all codes are distinct
  for (size_t i = 0; i < all_codes.size(); ++i) {
    for (size_t j = i + 1; j < all_codes.size(); ++j) {
      EXPECT_NE(all_codes[i], all_codes[j]) 
        << "Error codes at indices " << i << " and " << j << " are not distinct";
    }
  }
}

} // namespace roadmap_explorer

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

