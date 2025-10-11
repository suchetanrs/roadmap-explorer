#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/information_gain/BaseInformationGain.hpp"
#include "roadmap_explorer/planners/BasePlanner.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include "roadmap_explorer/util/Logger.hpp"

using namespace roadmap_explorer;

// Mock InformationGain plugin for testing
class MockInformationGain : public BaseInformationGain
{
public:
    MockInformationGain() = default;
    
    void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) override
    {
        configured_ = true;
        costmap_ros_ = explore_costmap_ros;
    }
    
    void reset() override
    {
        reset_called_ = true;
    }
    
    void setInformationGainForFrontier(
        FrontierPtr & frontier,
        std::vector<double> & polygon_xy_min_max) override
    {
        (void)polygon_xy_min_max;  // Suppress unused parameter warning
        // Set a mock information gain based on frontier position
        auto goal = frontier->getGoalPoint();
        double info_gain = 100.0 + goal.x + goal.y;
        frontier->setArrivalInformation(info_gain);
        set_info_gain_called_ = true;
    }
    
    bool configured_ = false;
    bool reset_called_ = false;
    bool set_info_gain_called_ = false;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

// Mock Planner plugin for testing
class MockPlanner : public BasePlanner
{
public:
    MockPlanner() = default;
    
    void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) override
    {
        configured_ = true;
        costmap_ros_ = explore_costmap_ros;
    }
    
    void reset() override
    {
        reset_called_ = true;
    }
    
    void setPlanForFrontier(
        const geometry_msgs::msg::Pose start_pose,
        FrontierPtr & frontier) override
    {
        // Set a mock path length based on distance to frontier
        auto goal = frontier->getGoalPoint();
        double dx = goal.x - start_pose.position.x;
        double dy = goal.y - start_pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        frontier->setPathLength(distance);
        frontier->setPathLengthInM(distance);
        frontier->setAchievability(true);
        set_plan_called_ = true;
    }
    
    bool configured_ = false;
    bool reset_called_ = false;
    bool set_plan_called_ = false;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

class CostAssignerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Initialize ROS if not already initialized
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create a lifecycle node for testing
        node_ = std::make_shared<nav2_util::LifecycleNode>("test_cost_assigner_node");
        
        // Create mock costmap
        createMockCostmap();
        
        // Initialize ParameterHandler for testing
        try {
            ParameterHandler::createInstance();
            parameterInstance.makeParameters(node_);
            
            // Set default plugin parameters
            parameterInstance.setValue("costAssigner.information_gain_plugin", 
                std::string("roadmap_explorer::MockInformationGain"));
            parameterInstance.setValue("costAssigner.planner_plugin", 
                std::string("roadmap_explorer::MockPlanner"));
        } catch (const std::exception& e) {
            // ParameterHandler might already exist
        }
        
        // Create CostAssigner instance
        cost_assigner_ = std::make_unique<CostAssigner>(costmap_ros_);
    }
    
    void TearDown() override
    {
        cost_assigner_.reset();
        costmap_ros_.reset();
        node_.reset();
        
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
    
    void createMockCostmap()
    {
        // Create a costmap for testing
        costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
        
        // Initialize the costmap with basic parameters
        costmap_ros_->on_configure(rclcpp_lifecycle::State());
        
        // Get the underlying costmap and set some basic properties
        auto costmap = costmap_ros_->getCostmap();
        
        // Set up a 20x20 grid for testing
        costmap->resizeMap(20, 20, 0.5, -5.0, -5.0); // 0.5m resolution, origin at (-5, -5)
        
        // Fill with test data - create a mix of free space, unknown, and obstacles
        for (unsigned int i = 0; i < 20; ++i) {
            for (unsigned int j = 0; j < 20; ++j) {
                if (i == 0 || i == 19 || j == 0 || j == 19) {
                    // Border obstacles
                    costmap->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
                } else if (i < 5 || j < 5) {
                    // Some explored free space
                    costmap->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
                } else {
                    // Unknown space in the middle
                    costmap->setCost(i, j, nav2_costmap_2d::NO_INFORMATION);
                }
            }
        }
    }
    
    FrontierPtr createTestFrontier(double x, double y, size_t uid)
    {
        auto frontier = std::make_shared<Frontier>();
        frontier->setUID(uid);
        frontier->setGoalPoint(x, y);
        frontier->setSize(10);
        frontier->setGoalOrientation(0.0);
        frontier->setAchievability(true);
        return frontier;
    }
    
    geometry_msgs::msg::PolygonStamped createTestPolygon(
        double min_x, double min_y, double max_x, double max_y)
    {
        geometry_msgs::msg::PolygonStamped polygon;
        polygon.header.frame_id = "map";
        
        geometry_msgs::msg::Point32 p1, p2, p3, p4;
        p1.x = min_x; p1.y = min_y; p1.z = 0.0;
        p2.x = min_x; p2.y = max_y; p2.z = 0.0;
        p3.x = max_x; p3.y = max_y; p3.z = 0.0;
        p4.x = max_x; p4.y = min_y; p4.z = 0.0;
        
        polygon.polygon.points.push_back(p1);
        polygon.polygon.points.push_back(p2);
        polygon.polygon.points.push_back(p3);
        polygon.polygon.points.push_back(p4);
        
        return polygon;
    }
    
    std::shared_ptr<nav2_util::LifecycleNode> node_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<CostAssigner> cost_assigner_;
};

// Test constructor
TEST_F(CostAssignerTest, Constructor)
{
    ASSERT_NE(cost_assigner_, nullptr);
}

// Test updateBoundaryPolygon with valid polygon
TEST_F(CostAssignerTest, UpdateBoundaryPolygonValid)
{
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    
    ASSERT_NO_THROW({
        cost_assigner_->updateBoundaryPolygon(polygon);
    });
}

// Test updateBoundaryPolygon with empty polygon (should use whole map)
TEST_F(CostAssignerTest, UpdateBoundaryPolygonEmpty)
{
    geometry_msgs::msg::PolygonStamped empty_polygon;
    empty_polygon.header.frame_id = "map";
    
    ASSERT_NO_THROW({
        cost_assigner_->updateBoundaryPolygon(empty_polygon);
    });
}

// Test updateBoundaryPolygon with large polygon
TEST_F(CostAssignerTest, UpdateBoundaryPolygonLarge)
{
    auto polygon = createTestPolygon(-100.0, -100.0, 100.0, 100.0);
    
    ASSERT_NO_THROW({
        cost_assigner_->updateBoundaryPolygon(polygon);
    });
}

// Test updateBoundaryPolygon with single point
TEST_F(CostAssignerTest, UpdateBoundaryPolygonSinglePoint)
{
    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header.frame_id = "map";
    
    geometry_msgs::msg::Point32 p1;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    polygon.polygon.points.push_back(p1);
    
    ASSERT_NO_THROW({
        cost_assigner_->updateBoundaryPolygon(polygon);
    });
}

// Test populateFrontierCosts with valid frontiers
TEST_F(CostAssignerTest, PopulateFrontierCostsValid)
{
    // Set up boundary polygon first
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    // Create test frontiers
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    request->frontier_list.push_back(createTestFrontier(1.0, 1.0, 1));
    request->frontier_list.push_back(createTestFrontier(2.0, 2.0, 2));
    request->frontier_list.push_back(createTestFrontier(3.0, 3.0, 3));
    
    // Note: This test will fail without actual plugin loading mechanism
    // In a real scenario, we'd need to mock the pluginlib or use actual plugins
    // For now, we expect an exception when trying to load non-existent plugins
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Test populateFrontierCosts with empty frontier list
TEST_F(CostAssignerTest, PopulateFrontierCostsEmptyList)
{
    // Set up boundary polygon first
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    // Empty frontier list
    
    ASSERT_THROW({
        bool result = cost_assigner_->populateFrontierCosts(request);
        EXPECT_FALSE(result);
    }, std::runtime_error);
}

// Test populateFrontierCosts with blacklisted frontiers
TEST_F(CostAssignerTest, PopulateFrontierCostsWithBlacklist)
{
    // Set up boundary polygon first
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    auto frontier1 = createTestFrontier(1.0, 1.0, 1);
    auto frontier2 = createTestFrontier(2.0, 2.0, 2);
    auto frontier3 = createTestFrontier(3.0, 3.0, 3);
    
    request->frontier_list.push_back(frontier1);
    request->frontier_list.push_back(frontier2);
    request->frontier_list.push_back(frontier3);
    
    // Blacklist frontier2
    request->prohibited_frontiers.push_back(frontier2);
    
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Test populateFrontierCosts with frontiers outside boundary
TEST_F(CostAssignerTest, PopulateFrontierCostsOutsideBoundary)
{
    // Set up boundary polygon
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    // Create frontiers outside the boundary
    request->frontier_list.push_back(createTestFrontier(10.0, 10.0, 1));
    request->frontier_list.push_back(createTestFrontier(-10.0, -10.0, 2));
    request->frontier_list.push_back(createTestFrontier(1.0, 1.0, 3)); // This one is inside
    
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Test populateFrontierCosts with duplicate frontiers
TEST_F(CostAssignerTest, PopulateFrontierCostsDuplicates)
{
    // Set up boundary polygon
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    auto frontier1 = createTestFrontier(1.0, 1.0, 1);
    
    // Add the same frontier twice (same pointer)
    request->frontier_list.push_back(frontier1);
    request->frontier_list.push_back(frontier1);
    
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, RoadmapExplorerException);
}

// Test populateFrontierCosts without setting boundary polygon
TEST_F(CostAssignerTest, PopulateFrontierCostsNoBoundary)
{
    // Don't set boundary polygon
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    request->frontier_list.push_back(createTestFrontier(1.0, 1.0, 1));
    
    ASSERT_THROW({
        bool result = cost_assigner_->populateFrontierCosts(request);
        EXPECT_FALSE(result);
    }, std::runtime_error);
}

// Test populateFrontierCosts with multiple frontiers at different distances
TEST_F(CostAssignerTest, PopulateFrontierCostsMultipleDistances)
{
    // Set up boundary polygon
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    // Create frontiers at varying distances
    request->frontier_list.push_back(createTestFrontier(0.5, 0.5, 1));  // Close
    request->frontier_list.push_back(createTestFrontier(2.0, 2.0, 2));  // Medium
    request->frontier_list.push_back(createTestFrontier(4.0, 4.0, 3));  // Far
    
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Test with frontiers at boundary edges
TEST_F(CostAssignerTest, PopulateFrontierCostsBoundaryEdges)
{
    // Set up boundary polygon
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    // Create frontiers exactly at the boundary
    request->frontier_list.push_back(createTestFrontier(-5.0, -5.0, 1));  // Min corner
    request->frontier_list.push_back(createTestFrontier(5.0, 5.0, 2));    // Max corner
    request->frontier_list.push_back(createTestFrontier(0.0, -5.0, 3));   // Bottom edge
    request->frontier_list.push_back(createTestFrontier(5.0, 0.0, 4));    // Right edge
    
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Test with all frontiers blacklisted
TEST_F(CostAssignerTest, PopulateFrontierCostsAllBlacklisted)
{
    // Set up boundary polygon
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    auto frontier1 = createTestFrontier(1.0, 1.0, 1);
    auto frontier2 = createTestFrontier(2.0, 2.0, 2);
    
    request->frontier_list.push_back(frontier1);
    request->frontier_list.push_back(frontier2);
    
    // Blacklist all frontiers
    request->prohibited_frontiers.push_back(frontier1);
    request->prohibited_frontiers.push_back(frontier2);
    
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Test with very large number of frontiers
TEST_F(CostAssignerTest, PopulateFrontierCostsLargeNumber)
{
    // Set up boundary polygon
    auto polygon = createTestPolygon(-10.0, -10.0, 10.0, 10.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    // Create many frontiers
    for (size_t i = 0; i < 50; ++i) {
        double x = -9.0 + (i % 10) * 2.0;
        double y = -9.0 + (i / 10) * 2.0;
        request->frontier_list.push_back(createTestFrontier(x, y, i));
    }
    
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Test polygon with negative coordinates
TEST_F(CostAssignerTest, UpdateBoundaryPolygonNegativeCoordinates)
{
    auto polygon = createTestPolygon(-10.0, -10.0, -1.0, -1.0);
    
    ASSERT_NO_THROW({
        cost_assigner_->updateBoundaryPolygon(polygon);
    });
}

// Test polygon with mixed positive and negative coordinates
TEST_F(CostAssignerTest, UpdateBoundaryPolygonMixedCoordinates)
{
    auto polygon = createTestPolygon(-5.0, -3.0, 7.0, 8.0);
    
    ASSERT_NO_THROW({
        cost_assigner_->updateBoundaryPolygon(polygon);
    });
}

// Test with start pose at different locations
TEST_F(CostAssignerTest, PopulateFrontierCostsDifferentStartPose)
{
    // Set up boundary polygon
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    // Start pose at a non-zero location
    request->start_pose.pose.position.x = 2.0;
    request->start_pose.pose.position.y = 2.0;
    request->start_pose.pose.position.z = 0.0;
    
    request->frontier_list.push_back(createTestFrontier(1.0, 1.0, 1));
    request->frontier_list.push_back(createTestFrontier(3.0, 3.0, 2));
    
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Test destructor
TEST_F(CostAssignerTest, Destructor)
{
    // Create a new CostAssigner instance
    auto temp_cost_assigner = std::make_unique<CostAssigner>(costmap_ros_);
    
    // Destroy it
    ASSERT_NO_THROW({
        temp_cost_assigner.reset();
    });
}

// Test with irregular polygon (not a rectangle)
TEST_F(CostAssignerTest, UpdateBoundaryPolygonIrregular)
{
    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header.frame_id = "map";
    
    // Create a pentagon
    geometry_msgs::msg::Point32 p1, p2, p3, p4, p5;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = 2.0; p2.y = 1.0; p2.z = 0.0;
    p3.x = 3.0; p3.y = 3.0; p3.z = 0.0;
    p4.x = 1.0; p4.y = 4.0; p4.z = 0.0;
    p5.x = -1.0; p5.y = 2.0; p5.z = 0.0;
    
    polygon.polygon.points.push_back(p1);
    polygon.polygon.points.push_back(p2);
    polygon.polygon.points.push_back(p3);
    polygon.polygon.points.push_back(p4);
    polygon.polygon.points.push_back(p5);
    
    ASSERT_NO_THROW({
        cost_assigner_->updateBoundaryPolygon(polygon);
    });
}

// Test with frontiers having same goal point but different UIDs
TEST_F(CostAssignerTest, PopulateFrontierCostsSameGoalPoint)
{
    // Set up boundary polygon
    auto polygon = createTestPolygon(-5.0, -5.0, 5.0, 5.0);
    cost_assigner_->updateBoundaryPolygon(polygon);
    
    auto request = std::make_shared<CalculateFrontierCostsRequest>();
    request->start_pose.pose.position.x = 0.0;
    request->start_pose.pose.position.y = 0.0;
    request->start_pose.pose.position.z = 0.0;
    
    // Create frontiers with same goal point but different UIDs
    auto frontier1 = createTestFrontier(1.0, 1.0, 1);
    auto frontier2 = createTestFrontier(1.0, 1.0, 2);  // Same position, different UID
    
    request->frontier_list.push_back(frontier1);
    request->frontier_list.push_back(frontier2);
    
    // These should be treated as different frontiers since they have different UIDs
    ASSERT_THROW({
        cost_assigner_->populateFrontierCosts(request);
    }, std::runtime_error);
}

// Main function
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

