#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "roadmap_explorer/frontier_search/BaseFrontierSearch.hpp"
#include "roadmap_explorer/Frontier.hpp"

using namespace roadmap_explorer;

// Test implementation of BaseFrontierSearch for testing the interface
class TestFrontierSearch : public FrontierSearchBase
{
public:
    TestFrontierSearch() = default;
    
    void configure(nav2_costmap_2d::Costmap2D * costmap) override
    {
        costmap_ = costmap;
    }
    
    void reset() override 
    {
        reset_called_ = true;
        test_frontiers_.clear();
    }
    
    bool setFrontierSearchDistance(double value) override 
    {
        set_distance_called_ = true;
        set_distance_value_ = value;
        if (value > max_distance_) {
            return false;
        }
        frontier_search_distance_ = value;
        return true;
    }
    
    FrontierSearchResult searchFrom(
        geometry_msgs::msg::Point position,
        std::vector<FrontierPtr> & output_frontier_list) override 
    {
        search_called_ = true;
        search_position_ = position;
        
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
    bool wasSetDistanceCalled() const { return set_distance_called_; }
    bool wasSearchCalled() const { return search_called_; }
    bool wasGetAllCalled() const { return get_all_called_; }
    double getSetDistanceValue() const { return set_distance_value_; }
    geometry_msgs::msg::Point getSearchPosition() const { return search_position_; }
    
    void setMaxDistance(double max_dist) { max_distance_ = max_dist; }
    void setInitialDistance(double init_dist) { 
        frontier_search_distance_ = init_dist;
    }
    void addTestFrontier(const std::vector<double>& frontier) {
        test_frontiers_.push_back(frontier);
    }

private:
    bool reset_called_ = false;
    bool set_distance_called_ = false;
    bool search_called_ = false;
    bool get_all_called_ = false;
    double set_distance_value_ = 0.0;
    double max_distance_ = 100.0;
    geometry_msgs::msg::Point search_position_;
    std::vector<std::vector<double>> test_frontiers_;
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
        
        // Create test costmap
        costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
            10, 10,  // width, height
            1.0,     // resolution
            0.0, 0.0 // origin_x, origin_y
        );
        
        // Initialize costmap with free space
        for (unsigned int x = 0; x < 10; ++x) {
            for (unsigned int y = 0; y < 10; ++y) {
                costmap_->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
            }
        }
        
        // Create test frontier search instance
        frontier_search_ = std::make_unique<TestFrontierSearch>();
    }
    
    void TearDown() override
    {
        frontier_search_.reset();
        costmap_.reset();
    }

    std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
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
    frontier_search_->configure(costmap_.get());
    
    // The base class configure should set the costmap pointer
    // We can't directly access it, but we can test that it doesn't crash
    EXPECT_NO_THROW(frontier_search_->configure(costmap_.get()));
}

// Test configure with nullptr
TEST_F(BaseFrontierSearchTest, ConfigureWithNullptr)
{
    EXPECT_NO_THROW(frontier_search_->configure(nullptr));
}

// Test reset method
TEST_F(BaseFrontierSearchTest, Reset)
{
    EXPECT_FALSE(frontier_search_->wasResetCalled());
    frontier_search_->reset();
    EXPECT_TRUE(frontier_search_->wasResetCalled());
}

// Test setFrontierSearchDistance method
TEST_F(BaseFrontierSearchTest, SetFrontierSearchDistance)
{
    frontier_search_->setInitialDistance(10.0);
    frontier_search_->setMaxDistance(50.0);
    
    EXPECT_FALSE(frontier_search_->wasSetDistanceCalled());
    
    // Test successful set
    bool result = frontier_search_->setFrontierSearchDistance(25.0);
    EXPECT_TRUE(result);
    EXPECT_TRUE(frontier_search_->wasSetDistanceCalled());
    EXPECT_DOUBLE_EQ(frontier_search_->getSetDistanceValue(), 25.0);
    EXPECT_DOUBLE_EQ(frontier_search_->getFrontierSearchDistance(), 25.0);
}

// Test setFrontierSearchDistance exceeding maximum
TEST_F(BaseFrontierSearchTest, SetFrontierSearchDistanceExceedsMax)
{
    frontier_search_->setInitialDistance(10.0);
    frontier_search_->setMaxDistance(20.0);
    
    // Test set that would exceed maximum
    bool result = frontier_search_->setFrontierSearchDistance(30.0);
    EXPECT_FALSE(result);
    EXPECT_TRUE(frontier_search_->wasSetDistanceCalled());
    EXPECT_DOUBLE_EQ(frontier_search_->getSetDistanceValue(), 30.0);
    // Distance should not have changed
    EXPECT_DOUBLE_EQ(frontier_search_->getFrontierSearchDistance(), 10.0);
}

// Test getFrontierSearchDistance method
TEST_F(BaseFrontierSearchTest, GetFrontierSearchDistance)
{
    frontier_search_->setInitialDistance(25.0);
    EXPECT_DOUBLE_EQ(frontier_search_->getFrontierSearchDistance(), 25.0);
    
    frontier_search_->setFrontierSearchDistance(35.0);
    EXPECT_DOUBLE_EQ(frontier_search_->getFrontierSearchDistance(), 35.0);
}

// Test searchFrom method with valid position
TEST_F(BaseFrontierSearchTest, SearchFromValidPosition)
{
    geometry_msgs::msg::Point position;
    position.x = 5.0;
    position.y = 5.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    
    EXPECT_FALSE(frontier_search_->wasSearchCalled());
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers);
    
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
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers);
    
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
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers);
    
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
    frontier_search_->setInitialDistance(10.0);
    frontier_search_->setMaxDistance(50.0);
    
    // Configure
    frontier_search_->configure(costmap_.get());
    
    // Reset
    frontier_search_->reset();
    EXPECT_TRUE(frontier_search_->wasResetCalled());
    
    // Set distance
    bool set_result = frontier_search_->setFrontierSearchDistance(25.0);
    EXPECT_TRUE(set_result);
    EXPECT_DOUBLE_EQ(frontier_search_->getFrontierSearchDistance(), 25.0);
    
    // Search for frontiers
    geometry_msgs::msg::Point position;
    position.x = 10.0;
    position.y = 10.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    FrontierSearchResult search_result = frontier_search_->searchFrom(position, output_frontiers);
    
    EXPECT_EQ(search_result, FrontierSearchResult::SUCCESSFUL_SEARCH);
    EXPECT_EQ(output_frontiers.size(), 2);
    
    // Get all frontiers
    auto all_frontiers = frontier_search_->getAllFrontiers();
    EXPECT_TRUE(frontier_search_->wasGetAllCalled());
    
    // Set different distance
    frontier_search_->setFrontierSearchDistance(15.0);
    EXPECT_DOUBLE_EQ(frontier_search_->getFrontierSearchDistance(), 15.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    
    int result = RUN_ALL_TESTS();
    
    rclcpp::shutdown();
    return result;
}
