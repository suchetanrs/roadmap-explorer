#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "roadmap_explorer/information_gain/BaseInformationGain.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/Parameters.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace roadmap_explorer
{

// Mock implementation of BaseInformationGain for testing
class MockInformationGain : public BaseInformationGain
{
public:
  MockInformationGain() = default;
  virtual ~MockInformationGain() = default;

  void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, 
                 std::string name, 
                 std::shared_ptr<nav2_util::LifecycleNode> node) override
  {
    configure_called_ = true;
    explore_costmap_ros_ = explore_costmap_ros;
    node_ = node;
    exploration_costmap_ = explore_costmap_ros->getCostmap();
    
    // Declare and get parameters
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_camera_depth", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".delta_theta", rclcpp::ParameterValue(0.10));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".camera_fov", rclcpp::ParameterValue(1.04));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".factor_of_max_is_min", rclcpp::ParameterValue(0.70));
    
    MAX_CAMERA_DEPTH = node->get_parameter(name + ".max_camera_depth").as_double();
    DELTA_THETA = node->get_parameter(name + ".delta_theta").as_double();
    CAMERA_FOV = node->get_parameter(name + ".camera_fov").as_double();
    factor_of_max_is_min = node->get_parameter(name + ".factor_of_max_is_min").as_double();
  }

  void reset() override
  {
    reset_called_ = true;
  }

  void setInformationGainForFrontier(
    FrontierPtr & frontier,
    std::vector<double> & polygon_xy_min_max) override
  {
    set_info_gain_called_ = true;
    last_frontier_ = frontier;
    last_polygon_ = polygon_xy_min_max;
    
    // Set some mock values
    frontier->setArrivalInformation(42.0);
    frontier->setGoalOrientation(1.57); // 90 degrees
  }

  // Test accessors
  bool configure_called_ = false;
  bool reset_called_ = false;
  bool set_info_gain_called_ = false;
  FrontierPtr last_frontier_;
  std::vector<double> last_polygon_;
  
  // Mock member variables for testing
  nav2_costmap_2d::Costmap2D* exploration_costmap_ = nullptr;
  double MAX_CAMERA_DEPTH = 0.0;
  double DELTA_THETA = 0.0;
  double CAMERA_FOV = 0.0;
  double factor_of_max_is_min = 0.0;
  
  // Expose members for testing
  nav2_costmap_2d::Costmap2D* getExplorationCostmap() const { return exploration_costmap_; }
  double getMaxCameraDepth() const { return MAX_CAMERA_DEPTH; }
  double getDeltaTheta() const { return DELTA_THETA; }
  double getCameraFov() const { return CAMERA_FOV; }
  double getFactorOfMaxIsMin() const { return factor_of_max_is_min; }
};

class BaseInformationGainTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    
    // Create a lifecycle node for testing
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_base_info_gain_node");
    
    // Create TF buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    
    // Create costmap for testing
    createMockCostmap();
    
    // Create the mock plugin
    plugin_ = std::make_unique<MockInformationGain>();
    
    // Initialize ParameterHandler for testing
    try {
      ParameterHandler::createInstance();
      parameterInstance.makeParameters(node_);
    } catch (const std::exception&) {
      // ParameterHandler might already exist, ignore
    }
  }

  void TearDown() override
  {
    plugin_.reset();
    costmap_ros_.reset();
    node_.reset();
    
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void createMockCostmap()
  {
    // Create a simple costmap for testing
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
    
    // Initialize the costmap with basic parameters
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    
    // Get the underlying costmap and set some basic properties
    auto costmap = costmap_ros_->getCostmap();
    
    // Set up a simple 10x10 grid for testing
    costmap->resizeMap(10, 10, 1.0, 0.0, 0.0);
    
    // Fill with some test data
    for (unsigned int i = 0; i < 10; ++i) {
      for (unsigned int j = 0; j < 10; ++j) {
        if (i == 0 || i == 9 || j == 0 || j == 9) {
          costmap->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
        } else {
          costmap->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
        }
      }
    }
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<MockInformationGain> plugin_;
};

// Test default constructor
TEST_F(BaseInformationGainTest, DefaultConstructor)
{
  auto info_gain = std::make_unique<MockInformationGain>();
  EXPECT_NE(info_gain, nullptr);
  EXPECT_FALSE(info_gain->configure_called_);
  EXPECT_FALSE(info_gain->reset_called_);
  EXPECT_FALSE(info_gain->set_info_gain_called_);
}

// Test configure method
TEST_F(BaseInformationGainTest, Configure)
{
  EXPECT_FALSE(plugin_->configure_called_);
  
  plugin_->configure(costmap_ros_, "test_plugin", node_);
  
  EXPECT_TRUE(plugin_->configure_called_);
  EXPECT_NE(plugin_->getExplorationCostmap(), nullptr);
}

// Test reset method
TEST_F(BaseInformationGainTest, Reset)
{
  EXPECT_FALSE(plugin_->reset_called_);
  
  plugin_->reset();
  
  EXPECT_TRUE(plugin_->reset_called_);
}

// Test updateParameters method
TEST_F(BaseInformationGainTest, UpdateParameters)
{
  plugin_->configure(costmap_ros_, "test_plugin", node_);
  
  // Parameters should be loaded after configure
  EXPECT_GT(plugin_->getMaxCameraDepth(), 0.0);
  EXPECT_GT(plugin_->getDeltaTheta(), 0.0);
  EXPECT_GT(plugin_->getCameraFov(), 0.0);
  EXPECT_GT(plugin_->getFactorOfMaxIsMin(), 0.0);
  EXPECT_LE(plugin_->getFactorOfMaxIsMin(), 1.0);
}

// Test setInformationGainForFrontier method
TEST_F(BaseInformationGainTest, SetInformationGainForFrontier)
{
  plugin_->configure(costmap_ros_, "test_plugin", node_);
  
  // Create a test frontier
  auto frontier = std::make_shared<Frontier>();
  geometry_msgs::msg::Point goal_point;
  goal_point.x = 5.0;
  goal_point.y = 5.0;
  goal_point.z = 0.0;
  frontier->setGoalPoint(goal_point);
  
  // Create polygon bounds
  std::vector<double> polygon_bounds = {0.0, 0.0, 10.0, 10.0}; // [x_min, y_min, x_max, y_max]
  
  EXPECT_FALSE(plugin_->set_info_gain_called_);
  
  plugin_->setInformationGainForFrontier(frontier, polygon_bounds);
  
  EXPECT_TRUE(plugin_->set_info_gain_called_);
  EXPECT_EQ(plugin_->last_frontier_, frontier);
  EXPECT_EQ(plugin_->last_polygon_, polygon_bounds);
  
  // Check that the frontier was modified
  EXPECT_DOUBLE_EQ(frontier->getArrivalInformation(), 42.0);
  // Note: getGoalOrientation() returns a quaternion, not a double
  // We just check that it was set (not null)
  EXPECT_NO_THROW(frontier->getGoalOrientation());
}

// Test virtual destructor
TEST_F(BaseInformationGainTest, VirtualDestructor)
{
  // This test ensures the virtual destructor works correctly
  std::unique_ptr<BaseInformationGain> base_ptr = std::make_unique<MockInformationGain>();
  
  // Should not crash when destroyed
  base_ptr.reset();
  EXPECT_EQ(base_ptr, nullptr);
}

// Test parameter access after configuration
TEST_F(BaseInformationGainTest, ParameterAccess)
{
  plugin_->configure(costmap_ros_, "test_plugin", node_);
  
  // Test that parameters are accessible and have reasonable values
  double max_depth = plugin_->getMaxCameraDepth();
  double delta_theta = plugin_->getDeltaTheta();
  double camera_fov = plugin_->getCameraFov();
  double factor = plugin_->getFactorOfMaxIsMin();
  
  EXPECT_GT(max_depth, 0.0);
  EXPECT_LT(max_depth, 1000.0); // Reasonable upper bound
  
  EXPECT_GT(delta_theta, 0.0);
  EXPECT_LT(delta_theta, 2 * M_PI); // Should be less than full circle
  
  EXPECT_GT(camera_fov, 0.0);
  EXPECT_LT(camera_fov, 2 * M_PI); // Should be less than full circle
  
  EXPECT_GT(factor, 0.0);
  EXPECT_LE(factor, 1.0);
}

// Test multiple configure calls
TEST_F(BaseInformationGainTest, MultipleConfigure)
{
  plugin_->configure(costmap_ros_, "test_plugin", node_);
  EXPECT_TRUE(plugin_->configure_called_);
  
  // Reset the flag and configure again
  plugin_->configure_called_ = false;
  plugin_->configure(costmap_ros_, "test_plugin", node_);
  EXPECT_TRUE(plugin_->configure_called_);
}

// Test frontier with different positions
TEST_F(BaseInformationGainTest, FrontierDifferentPositions)
{
  plugin_->configure(costmap_ros_, "test_plugin", node_);
  
  std::vector<geometry_msgs::msg::Point> test_points;
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = 1.0; p1.y = 1.0; p1.z = 0.0;
  p2.x = 5.0; p2.y = 5.0; p2.z = 0.0;
  p3.x = 8.0; p3.y = 8.0; p3.z = 0.0;
  test_points = {p1, p2, p3};
  
  std::vector<double> polygon_bounds = {0.0, 0.0, 10.0, 10.0};
  
  for (const auto& point : test_points) {
    auto frontier = std::make_shared<Frontier>();
    frontier->setGoalPoint(point);
    
    plugin_->set_info_gain_called_ = false;
    plugin_->setInformationGainForFrontier(frontier, polygon_bounds);
    
    EXPECT_TRUE(plugin_->set_info_gain_called_);
    EXPECT_EQ(plugin_->last_frontier_, frontier);
    
    // Check that information was set
    EXPECT_DOUBLE_EQ(frontier->getArrivalInformation(), 42.0);
    // Note: getGoalOrientation() returns a quaternion, not a double
    EXPECT_NO_THROW(frontier->getGoalOrientation());
  }
}

// Test with different polygon bounds
TEST_F(BaseInformationGainTest, DifferentPolygonBounds)
{
  plugin_->configure(costmap_ros_, "test_plugin", node_);
  
  auto frontier = std::make_shared<Frontier>();
  geometry_msgs::msg::Point goal_point;
  goal_point.x = 5.0;
  goal_point.y = 5.0;
  goal_point.z = 0.0;
  frontier->setGoalPoint(goal_point);
  
  std::vector<std::vector<double>> test_bounds = {
    {0.0, 0.0, 10.0, 10.0},
    {-5.0, -5.0, 15.0, 15.0},
    {2.0, 2.0, 8.0, 8.0}
  };
  
  for (const auto& bounds : test_bounds) {
    plugin_->set_info_gain_called_ = false;
    auto bounds_copy = bounds;
    plugin_->setInformationGainForFrontier(frontier, bounds_copy);
    
    EXPECT_TRUE(plugin_->set_info_gain_called_);
    EXPECT_EQ(plugin_->last_polygon_, bounds);
  }
}

}  // namespace roadmap_explorer

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
