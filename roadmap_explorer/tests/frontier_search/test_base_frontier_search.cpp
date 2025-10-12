#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "roadmap_explorer/frontier_search/BaseFrontierSearch.hpp"
#include "roadmap_explorer/Frontier.hpp"

using namespace roadmap_explorer;

// Test implementation of BaseFrontierSearch for testing the interface
class TestFrontierSearch : public FrontierSearchBase
{
public:
    TestFrontierSearch() = default;
    
    void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, 
                   std::string /*name*/, 
                   std::shared_ptr<nav2_util::LifecycleNode> node) override
    {
        explore_costmap_ros_ = explore_costmap_ros;
        node_ = node;
        costmap_ = explore_costmap_ros ? explore_costmap_ros->getCostmap() : nullptr;
    }
    
    void reset() override 
    {
        reset_called_ = true;
        test_frontiers_.clear();
    }
    
    FrontierSearchResult searchFrom(
        geometry_msgs::msg::Point position,
        std::vector<FrontierPtr> & output_frontier_list,
        double max_frontier_search_distance) override 
    {
        search_called_ = true;
        search_position_ = position;
        max_search_distance_used_ = max_frontier_search_distance;
        
        // Simulate different search results based on position
        if (position.x < 0 || position.y < 0) {
            return FrontierSearchResult::ROBOT_OUT_OF_BOUNDS;
        }
        
        if (position.x > 100 || position.y > 100) {
            return FrontierSearchResult::CANNOT_FIND_CELL_TO_SEARCH;
        }
        
        // Create test frontiers
        auto frontier1 = std::make_shared<Frontier>();
        frontier1->setGoalPoint(position.x + 1.0, position.y + 1.0);
        frontier1->setSize(5);
        frontier1->setUID(1);
        
        auto frontier2 = std::make_shared<Frontier>();
        frontier2->setGoalPoint(position.x + 2.0, position.y + 2.0);
        frontier2->setSize(10);
        frontier2->setUID(2);
        
        output_frontier_list.push_back(frontier1);
        output_frontier_list.push_back(frontier2);
        
        return FrontierSearchResult::SUCCESSFUL_SEARCH;
    }
    
    std::vector<std::vector<double>> getAllFrontiers() override 
    {
        get_all_called_ = true;
        return test_frontiers_;
    }
    
    // Test helper methods
    bool wasResetCalled() const { return reset_called_; }
    bool wasSearchCalled() const { return search_called_; }
    bool wasGetAllCalled() const { return get_all_called_; }
    geometry_msgs::msg::Point getSearchPosition() const { return search_position_; }
    double getMaxSearchDistanceUsed() const { return max_search_distance_used_; }
    
    void addTestFrontier(const std::vector<double>& frontier) {
        test_frontiers_.push_back(frontier);
    }

private:
    bool reset_called_ = false;
    bool search_called_ = false;
    bool get_all_called_ = false;
    double max_search_distance_used_ = 0.0;
    geometry_msgs::msg::Point search_position_;
    std::vector<std::vector<double>> test_frontiers_;
    nav2_costmap_2d::Costmap2D* costmap_ = nullptr;
};

class BaseFrontierSearchTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Initialize ROS if not already initialized
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create lifecycle node for testing
        node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");
        
        // Create test costmap ROS wrapper
        // Note: In a real scenario, this would be properly initialized with tf, etc.
        // For testing purposes, we'll just create the frontier search without full costmap_ros
        
        // Create test frontier search instance
        frontier_search_ = std::make_unique<TestFrontierSearch>();
    }
    
    void TearDown() override
    {
        frontier_search_.reset();
        node_.reset();
    }

    std::shared_ptr<nav2_util::LifecycleNode> node_;
    std::unique_ptr<TestFrontierSearch> frontier_search_;
};

// Test default constructor
TEST_F(BaseFrontierSearchTest, DefaultConstructor)
{
    auto search = std::make_unique<TestFrontierSearch>();
    EXPECT_NE(search, nullptr);
}

// Test configure method
TEST_F(BaseFrontierSearchTest, Configure)
{
    // The configure method should not crash with valid inputs
    EXPECT_NO_THROW(frontier_search_->configure(nullptr, "test_frontier_search", node_));
}

// Test configure with nullptr
TEST_F(BaseFrontierSearchTest, ConfigureWithNullptr)
{
    EXPECT_NO_THROW(frontier_search_->configure(nullptr, "test", nullptr));
}

// Test reset method
TEST_F(BaseFrontierSearchTest, Reset)
{
    EXPECT_FALSE(frontier_search_->wasResetCalled());
    frontier_search_->reset();
    EXPECT_TRUE(frontier_search_->wasResetCalled());
}

// Test searchFrom with max distance parameter
TEST_F(BaseFrontierSearchTest, SearchFromWithMaxDistance)
{
    geometry_msgs::msg::Point position;
    position.x = 5.0;
    position.y = 5.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 25.0;
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_TRUE(frontier_search_->wasSearchCalled());
    EXPECT_EQ(result, FrontierSearchResult::SUCCESSFUL_SEARCH);
    EXPECT_DOUBLE_EQ(frontier_search_->getMaxSearchDistanceUsed(), 25.0);
}

// Test searchFrom method with valid position
TEST_F(BaseFrontierSearchTest, SearchFromValidPosition)
{
    geometry_msgs::msg::Point position;
    position.x = 5.0;
    position.y = 5.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    
    EXPECT_FALSE(frontier_search_->wasSearchCalled());
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_TRUE(frontier_search_->wasSearchCalled());
    EXPECT_EQ(result, FrontierSearchResult::SUCCESSFUL_SEARCH);
    EXPECT_EQ(output_frontiers.size(), 2);
    
    // Check search position was stored correctly
    auto stored_pos = frontier_search_->getSearchPosition();
    EXPECT_DOUBLE_EQ(stored_pos.x, 5.0);
    EXPECT_DOUBLE_EQ(stored_pos.y, 5.0);
    EXPECT_DOUBLE_EQ(stored_pos.z, 0.0);
    
    // Check frontier properties
    EXPECT_DOUBLE_EQ(output_frontiers[0]->getGoalPoint().x, 6.0);
    EXPECT_DOUBLE_EQ(output_frontiers[0]->getGoalPoint().y, 6.0);
    EXPECT_EQ(output_frontiers[0]->getSize(), 5);
    EXPECT_EQ(output_frontiers[0]->getUID(), 1);
    
    EXPECT_DOUBLE_EQ(output_frontiers[1]->getGoalPoint().x, 7.0);
    EXPECT_DOUBLE_EQ(output_frontiers[1]->getGoalPoint().y, 7.0);
    EXPECT_EQ(output_frontiers[1]->getSize(), 10);
    EXPECT_EQ(output_frontiers[1]->getUID(), 2);
}

// Test searchFrom method with out of bounds position
TEST_F(BaseFrontierSearchTest, SearchFromOutOfBounds)
{
    geometry_msgs::msg::Point position;
    position.x = -1.0;
    position.y = 5.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_TRUE(frontier_search_->wasSearchCalled());
    EXPECT_EQ(result, FrontierSearchResult::ROBOT_OUT_OF_BOUNDS);
    EXPECT_TRUE(output_frontiers.empty());
}

// Test searchFrom method with cannot find cell position
TEST_F(BaseFrontierSearchTest, SearchFromCannotFindCell)
{
    geometry_msgs::msg::Point position;
    position.x = 150.0;
    position.y = 150.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_TRUE(frontier_search_->wasSearchCalled());
    EXPECT_EQ(result, FrontierSearchResult::CANNOT_FIND_CELL_TO_SEARCH);
    EXPECT_TRUE(output_frontiers.empty());
}

// Test getAllFrontiers method
TEST_F(BaseFrontierSearchTest, GetAllFrontiers)
{
    // Add test frontiers
    frontier_search_->addTestFrontier({1.0, 2.0});
    frontier_search_->addTestFrontier({3.0, 4.0});
    frontier_search_->addTestFrontier({5.0, 6.0});
    
    EXPECT_FALSE(frontier_search_->wasGetAllCalled());
    
    auto all_frontiers = frontier_search_->getAllFrontiers();
    
    EXPECT_TRUE(frontier_search_->wasGetAllCalled());
    EXPECT_EQ(all_frontiers.size(), 3);
    
    EXPECT_DOUBLE_EQ(all_frontiers[0][0], 1.0);
    EXPECT_DOUBLE_EQ(all_frontiers[0][1], 2.0);
    EXPECT_DOUBLE_EQ(all_frontiers[1][0], 3.0);
    EXPECT_DOUBLE_EQ(all_frontiers[1][1], 4.0);
    EXPECT_DOUBLE_EQ(all_frontiers[2][0], 5.0);
    EXPECT_DOUBLE_EQ(all_frontiers[2][1], 6.0);
}

// Test getAllFrontiers method with empty list
TEST_F(BaseFrontierSearchTest, GetAllFrontiersEmpty)
{
    auto all_frontiers = frontier_search_->getAllFrontiers();
    
    EXPECT_TRUE(frontier_search_->wasGetAllCalled());
    EXPECT_TRUE(all_frontiers.empty());
}

// Test virtual destructor
TEST_F(BaseFrontierSearchTest, VirtualDestructor)
{
    // Test that we can delete through base pointer
    std::unique_ptr<FrontierSearchBase> base_ptr = std::make_unique<TestFrontierSearch>();
    EXPECT_NO_THROW(base_ptr.reset());
}

// Test FrontierSearchResult enum values
TEST_F(BaseFrontierSearchTest, FrontierSearchResultEnumValues)
{
    EXPECT_EQ(static_cast<int>(FrontierSearchResult::ROBOT_OUT_OF_BOUNDS), 0);
    EXPECT_EQ(static_cast<int>(FrontierSearchResult::CANNOT_FIND_CELL_TO_SEARCH), 1);
    EXPECT_EQ(static_cast<int>(FrontierSearchResult::SUCCESSFUL_SEARCH), 2);
}

// Test multiple operations in sequence
TEST_F(BaseFrontierSearchTest, MultipleOperationsSequence)
{
    // Configure
    frontier_search_->configure(nullptr, "test_frontier_search", node_);
    
    // Reset
    frontier_search_->reset();
    EXPECT_TRUE(frontier_search_->wasResetCalled());
    
    // Search for frontiers with a specific max distance
    geometry_msgs::msg::Point position;
    position.x = 10.0;
    position.y = 10.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 25.0;
    FrontierSearchResult search_result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_EQ(search_result, FrontierSearchResult::SUCCESSFUL_SEARCH);
    EXPECT_EQ(output_frontiers.size(), 2);
    EXPECT_DOUBLE_EQ(frontier_search_->getMaxSearchDistanceUsed(), 25.0);
    
    // Get all frontiers
    auto all_frontiers = frontier_search_->getAllFrontiers();
    EXPECT_TRUE(frontier_search_->wasGetAllCalled());
    
    // Search with different distance
    output_frontiers.clear();
    double new_max_distance = 15.0;
    frontier_search_->searchFrom(position, output_frontiers, new_max_distance);
    EXPECT_DOUBLE_EQ(frontier_search_->getMaxSearchDistanceUsed(), 15.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    
    int result = RUN_ALL_TESTS();
    
    rclcpp::shutdown();
    return result;
}
