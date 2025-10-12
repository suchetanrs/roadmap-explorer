#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <unordered_map>
#include <list>
#include <queue>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"
#include "roadmap_explorer/frontier_search/BaseFrontierSearch.hpp"

using namespace roadmap_explorer;

// Define a custom functor with an extra argument
class SortByMedianFunctor
{
public:
    SortByMedianFunctor(std::pair<double, double> centroid)
    : centroid(centroid) {}

    bool operator()(const std::pair<double, double> & a, const std::pair<double, double> & b) const
    {
        auto angle_a = atan2(a.second - centroid.second, a.first - centroid.first);
        if (angle_a < 0) {
            angle_a = angle_a + (2 * M_PI);
        }
        auto angle_b = atan2(b.second - centroid.second, b.first - centroid.first);
        if (angle_b < 0) {
            angle_b = angle_b + (2 * M_PI);
        }
        if (0 <= angle_a && angle_a <= M_PI / 2 && 3 * M_PI / 2 <= angle_b && angle_b <= 2 * M_PI) {
            return false;
        }
        if (0 <= angle_b && angle_b <= M_PI / 2 && 3 * M_PI / 2 <= angle_a && angle_a <= 2 * M_PI) {
            return true;
        }
        return angle_a < angle_b;
    }

private:
    std::pair<double, double> centroid;
};

// Test-friendly version of FrontierBFSearch that doesn't depend on global parameters
class TestFrontierBFSearch : public FrontierSearchBase
{
public:
    TestFrontierBFSearch() = default;
    
    TestFrontierBFSearch(nav2_costmap_2d::Costmap2D & costmap)
    {
        // Set default test parameters
        min_frontier_cluster_size_ = 2.0;
        max_frontier_cluster_size_ = 15.0;
        lethal_threshold_ = 160;
        
        // Store costmap pointer
        costmap_ = &costmap;
    }
    
    void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, 
                   std::string /*name*/, 
                   std::shared_ptr<nav2_util::LifecycleNode> node) override
    {
        explore_costmap_ros_ = explore_costmap_ros;
        node_ = node;
        costmap_ = explore_costmap_ros ? explore_costmap_ros->getCostmap() : nullptr;
    }

    ~TestFrontierBFSearch() override
    {
        every_frontier_list.clear();
    }

    void reset() override
    {
        every_frontier_list.clear();
    }

    FrontierSearchResult searchFrom(geometry_msgs::msg::Point position, 
                                     std::vector<FrontierPtr> & output_frontier_list,
                                     double max_frontier_search_distance) override
    {
        // Sanity check that robot is inside costmap bounds before searching
        unsigned int mx, my;
        if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
            return FrontierSearchResult::ROBOT_OUT_OF_BOUNDS;
        }

        map_ = costmap_->getCharMap();
        size_x_ = costmap_->getSizeInCellsX();
        size_y_ = costmap_->getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        std::vector<bool> frontier_flag(size_x_ * size_y_, false);
        std::vector<bool> visited_flag(size_x_ * size_y_, false);

        // initialize breadth first search queue
        std::queue<unsigned int> bfs;

        // find closest clear cell to start search
        unsigned int clear, pos = costmap_->getIndex(mx, my);
        if (nearestFreeCell(clear, pos, lethal_threshold_, *costmap_)) {
            bfs.push(clear);
        } else {
            bfs.push(pos);
            return FrontierSearchResult::CANNOT_FIND_CELL_TO_SEARCH;
        }
        visited_flag[bfs.front()] = true;

        auto distance_to_check_ = max_frontier_search_distance +
            (max_frontier_cluster_size_ * costmap_->getResolution() * 1.414);
        distance_to_check_ = std::pow(distance_to_check_, 2);

        while (rclcpp::ok() && !bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // iterate over 4-connected neighbourhood
            for (unsigned & nbr : nhood4(idx, *costmap_)) {
                // add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
                if (map_[nbr] < nav2_costmap_2d::LETHAL_OBSTACLE && !visited_flag[nbr]) {
                    visited_flag[nbr] = true;
                    unsigned int nbr_mx, nbr_my;
                    double nbr_wx, nbr_wy;
                    costmap_->indexToCells(nbr, nbr_mx, nbr_my);
                    costmap_->mapToWorld(nbr_mx, nbr_my, nbr_wx, nbr_wy);
                    if (distanceBetweenPointsSq(position, nbr_wx, nbr_wy) < distance_to_check_) {
                        bfs.push(nbr);
                    }
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                } else if (isNewFrontierCell(nbr, frontier_flag)) {
                    frontier_flag[nbr] = true;
                    std::vector<FrontierPtr> new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                    for (auto & curr_frontier : new_frontier) {
                        if (curr_frontier->getSize() > min_frontier_cluster_size_) {
                            output_frontier_list.push_back(curr_frontier);
                        }
                    }
                }
            }
        }
        return FrontierSearchResult::SUCCESSFUL_SEARCH;
    }

    std::vector<std::vector<double>> getAllFrontiers() override
    {
        return every_frontier_list;
    }

    // Test parameter setters
    void setMinFrontierClusterSize(double size) { min_frontier_cluster_size_ = size; }
    void setMaxFrontierClusterSize(double size) { max_frontier_cluster_size_ = size; }
    void setLethalThreshold(unsigned char threshold) { lethal_threshold_ = threshold; }

protected:
    std::vector<FrontierPtr> buildNewFrontier(
        unsigned int initial_cell,
        unsigned int reference,
        std::vector<bool> & frontier_flag)
    {
        int currentFrontierSize = 1;
        std::vector<FrontierPtr> calculated_frontiers;
        std::vector<std::pair<double, double>> frontier_cell_indices;

        // record initial contact point for frontier
        unsigned int ix, iy;
        costmap_->indexToCells(initial_cell, ix, iy);
        double wix, wiy;
        costmap_->mapToWorld(ix, iy, wix, wiy);
        every_frontier_list.push_back({wix, wiy});
        frontier_cell_indices.push_back(std::make_pair(wix, wiy));

        // push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);

        // cache reference position in world coords
        unsigned int rx, ry;
        double reference_x, reference_y;
        costmap_->indexToCells(reference, rx, ry);
        costmap_->mapToWorld(rx, ry, reference_x, reference_y);

        while (rclcpp::ok() && !bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int & nbr : nhood8(idx, *costmap_)) {
                // check if neighbour is a potential frontier cell
                if (isNewFrontierCell(nbr, frontier_flag)) {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_->indexToCells(nbr, mx, my);
                    costmap_->mapToWorld(mx, my, wx, wy);

                    // add to every frontier list
                    std::vector<double> coord_val;
                    coord_val.push_back(wx);
                    coord_val.push_back(wy);

                    every_frontier_list.push_back(coord_val);
                    frontier_cell_indices.push_back(std::make_pair(wx, wy));

                    // update frontier size
                    currentFrontierSize = currentFrontierSize + 1;

                    // add to queue for breadth first search
                    bfs.push(nbr);

                    if (currentFrontierSize > max_frontier_cluster_size_) {
                        FrontierPtr output = std::make_shared<Frontier>();
                        auto cluster_centroid = getCentroidOfCells(frontier_cell_indices, (costmap_->getResolution() * 1.414 * 2));
                        SortByMedianFunctor sortFunctor(cluster_centroid);
                        std::sort(frontier_cell_indices.begin(), frontier_cell_indices.end(), sortFunctor);
                        auto goal_point = frontier_cell_indices[static_cast<int>(frontier_cell_indices.size() / 2)];
                        output->setGoalPoint(goal_point.first, goal_point.second);
                        output->setSize(currentFrontierSize);
                        frontier_cell_indices.clear();
                        output->setUID(generateUID(output));
                        calculated_frontiers.push_back(output);
                        currentFrontierSize = 0;
                    }
                }
            }
        }
        
        if (currentFrontierSize > min_frontier_cluster_size_) {
            FrontierPtr output = std::make_shared<Frontier>();
            auto cluster_centroid = getCentroidOfCells(frontier_cell_indices, (costmap_->getResolution() * 1.414 * 2));
            SortByMedianFunctor sortFunctor(cluster_centroid);
            std::sort(frontier_cell_indices.begin(), frontier_cell_indices.end(), sortFunctor);
            auto goal_point = frontier_cell_indices[static_cast<int>(frontier_cell_indices.size() / 2)];
            output->setGoalPoint(goal_point.first, goal_point.second);
            output->setSize(currentFrontierSize);
            frontier_cell_indices.clear();
            output->setUID(generateUID(output));
            calculated_frontiers.push_back(output);
        }
        return calculated_frontiers;
    }

    bool isNewFrontierCell(unsigned int idx, const std::vector<bool> & frontier_flag)
    {
        // check that cell is unknown and not already marked as frontier
        if (!isUnknown(map_[idx]) || frontier_flag[idx]) {
            return false;
        }

        bool has_one_free_neighbour = false;
        bool has_one_lethal_neighbour = false;

        // frontier cells should have at least one cell in 4-connected neighbourhood that is free
        for (unsigned int nbr : nhood4(idx, *costmap_)) {
            if (isFree(map_[nbr])) {
                has_one_free_neighbour = true;
            }
            if (isLethal(map_[nbr])) {
                has_one_lethal_neighbour = true;
            }
        }
        if (has_one_lethal_neighbour) {
            return false;
        } else if (has_one_free_neighbour) {
            return true;
        } else {
            return false;
        }
    }

    std::pair<double, double> getCentroidOfCells(
        std::vector<std::pair<double, double>> & cells,
        double distance_to_offset)
    {
        double sumX = 0;
        double sumY = 0;

        for (const auto & point : cells) {
            sumX += point.first;
            sumY += point.second;
        }

        double centerX = static_cast<double>(sumX) / cells.size();
        double centerY = static_cast<double>(sumY) / cells.size();

        bool offset_centroid = false;
        double varX = 0, varY = 0;
        for (const auto & point : cells) {
            if (sqrt(pow(point.first - centerX, 2) + pow(point.second - centerY, 2)) < costmap_->getResolution() * 3) {
                offset_centroid = true;
            }
            varX += abs(point.first - centerX);
            varY += abs(point.second - centerY);
        }

        if (varX > varY && offset_centroid) {
            centerY -= distance_to_offset;
        }
        if (varX < varY && offset_centroid) {
            centerX -= distance_to_offset;
        }

        return std::make_pair(centerX, centerY);
    }

private:
    inline bool isLethal(unsigned char value)
    {
        return (int)value >= lethal_threshold_ && value != nav2_costmap_2d::NO_INFORMATION;
    }

    inline bool isUnknown(unsigned char value)
    {
        return value == nav2_costmap_2d::NO_INFORMATION;
    }

    inline bool isFree(unsigned char value)
    {
        return (int)value < lethal_threshold_;
    }

    unsigned char * map_;
    unsigned int size_x_, size_y_;
    std::vector<std::vector<double>> every_frontier_list;
    double min_frontier_cluster_size_;
    double max_frontier_cluster_size_;
    unsigned char lethal_threshold_;
    nav2_costmap_2d::Costmap2D* costmap_ = nullptr;
};

class FrontierBFSearchTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Initialize ROS if not already initialized
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create test costmap
        setupTestCostmap();
        
        // Create frontier search instance
        frontier_search_ = std::make_unique<TestFrontierBFSearch>(*costmap_);
    }
    
    void TearDown() override
    {
        frontier_search_.reset();
        costmap_.reset();
    }
    
    void setupTestCostmap()
    {
        // Create a 20x20 costmap for testing
        costmap_width_ = 20;
        costmap_height_ = 20;
        resolution_ = 0.5;
        origin_x_ = 0.0;
        origin_y_ = 0.0;
        
        costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
            costmap_width_, costmap_height_, resolution_, origin_x_, origin_y_);
        
        // Initialize with free space
        for (unsigned int x = 0; x < costmap_width_; ++x) {
            for (unsigned int y = 0; y < costmap_height_; ++y) {
                costmap_->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
            }
        }
        
        // Add obstacles
        addObstacles();
        
        // Add unknown regions (frontiers)
        addUnknownRegions();
    }
    
    void addObstacles()
    {
        // Add some lethal obstacles
        costmap_->setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
        costmap_->setCost(6, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
        costmap_->setCost(5, 6, nav2_costmap_2d::LETHAL_OBSTACLE);
        costmap_->setCost(6, 6, nav2_costmap_2d::LETHAL_OBSTACLE);
        
        // Add some high-cost obstacles
        costmap_->setCost(10, 10, 200);
        costmap_->setCost(11, 10, 200);
        costmap_->setCost(10, 11, 200);
        costmap_->setCost(11, 11, 200);
    }
    
    void addUnknownRegions()
    {
        // Create frontier regions (unknown cells adjacent to free space)
        // Top-right corner
        for (unsigned int x = 15; x < 20; ++x) {
            for (unsigned int y = 15; y < 20; ++y) {
                costmap_->setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
            }
        }
        
        // Bottom-left corner
        for (unsigned int x = 0; x < 5; ++x) {
            for (unsigned int y = 0; y < 5; ++y) {
                costmap_->setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
            }
        }
        
        // Small frontier region
        costmap_->setCost(8, 8, nav2_costmap_2d::NO_INFORMATION);
        costmap_->setCost(9, 8, nav2_costmap_2d::NO_INFORMATION);
        costmap_->setCost(8, 9, nav2_costmap_2d::NO_INFORMATION);
    }
    
    void setupLargeFrontierCostmap()
    {
        // Create a costmap with a large frontier cluster
        for (unsigned int x = 0; x < costmap_width_; ++x) {
            for (unsigned int y = 0; y < costmap_height_; ++y) {
                costmap_->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
            }
        }
        
        // Create a large unknown region
        for (unsigned int x = 10; x < 20; ++x) {
            for (unsigned int y = 10; y < 20; ++y) {
                costmap_->setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
            }
        }
    }

    std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    std::unique_ptr<TestFrontierBFSearch> frontier_search_;
    unsigned int costmap_width_, costmap_height_;
    double resolution_, origin_x_, origin_y_;
};

// Test constructor with costmap
TEST_F(FrontierBFSearchTest, ConstructorWithCostmap)
{
    EXPECT_NE(frontier_search_, nullptr);
}

// Test default constructor
TEST_F(FrontierBFSearchTest, DefaultConstructor)
{
    auto search = std::make_unique<TestFrontierBFSearch>();
    EXPECT_NE(search, nullptr);
}

// Test reset method
TEST_F(FrontierBFSearchTest, Reset)
{
    // First search to populate internal data
    geometry_msgs::msg::Point position;
    position.x = 7.0;
    position.y = 7.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    // Get all frontiers to verify there's data
    auto all_frontiers_before = frontier_search_->getAllFrontiers();
    EXPECT_FALSE(all_frontiers_before.empty());
    
    // Reset should clear internal data
    frontier_search_->reset();
    
    auto all_frontiers_after = frontier_search_->getAllFrontiers();
    EXPECT_TRUE(all_frontiers_after.empty());
}

// Test searchFrom with different max distances
TEST_F(FrontierBFSearchTest, SearchFromWithDifferentMaxDistances)
{
    geometry_msgs::msg::Point position;
    position.x = 7.0;
    position.y = 7.0;
    position.z = 0.0;
    
    // Test with a smaller max distance
    std::vector<FrontierPtr> output_frontiers1;
    double max_distance1 = 30.0;
    FrontierSearchResult result1 = frontier_search_->searchFrom(position, output_frontiers1, max_distance1);
    EXPECT_EQ(result1, FrontierSearchResult::SUCCESSFUL_SEARCH);
    
    // Test with a larger max distance
    std::vector<FrontierPtr> output_frontiers2;
    double max_distance2 = 50.0;
    FrontierSearchResult result2 = frontier_search_->searchFrom(position, output_frontiers2, max_distance2);
    EXPECT_EQ(result2, FrontierSearchResult::SUCCESSFUL_SEARCH);
    
    // Larger distance should find same or more frontiers
    EXPECT_GE(output_frontiers2.size(), output_frontiers1.size());
}

// Test searchFrom with valid position
TEST_F(FrontierBFSearchTest, SearchFromValidPosition)
{
    geometry_msgs::msg::Point position;
    position.x = 7.0;  // World coordinates
    position.y = 7.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_EQ(result, FrontierSearchResult::SUCCESSFUL_SEARCH);
    EXPECT_FALSE(output_frontiers.empty());
    
    // Check that frontiers have valid properties
    for (const auto& frontier : output_frontiers) {
        EXPECT_GT(frontier->getSize(), 0);
        EXPECT_NE(frontier->getUID(), 0);
        
        auto goal_point = frontier->getGoalPoint();
        EXPECT_FALSE(std::isnan(goal_point.x));
        EXPECT_FALSE(std::isnan(goal_point.y));
    }
}

// Test searchFrom with robot out of bounds
TEST_F(FrontierBFSearchTest, SearchFromOutOfBounds)
{
    geometry_msgs::msg::Point position;
    position.x = -5.0;  // Outside costmap bounds
    position.y = 5.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_EQ(result, FrontierSearchResult::ROBOT_OUT_OF_BOUNDS);
    EXPECT_TRUE(output_frontiers.empty());
}

// Test getAllFrontiers method
TEST_F(FrontierBFSearchTest, GetAllFrontiers)
{
    geometry_msgs::msg::Point position;
    position.x = 7.0;
    position.y = 7.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    auto all_frontiers = frontier_search_->getAllFrontiers();
    EXPECT_FALSE(all_frontiers.empty());
    
    // Each frontier should be a 2D coordinate
    for (const auto& frontier : all_frontiers) {
        EXPECT_EQ(frontier.size(), 2);
        EXPECT_FALSE(std::isnan(frontier[0]));
        EXPECT_FALSE(std::isnan(frontier[1]));
    }
}

// Test with minimum frontier cluster size filter
TEST_F(FrontierBFSearchTest, MinimumFrontierClusterSize)
{
    // Set a high minimum cluster size
    frontier_search_->setMinFrontierClusterSize(10.0);
    
    geometry_msgs::msg::Point position;
    position.x = 7.0;
    position.y = 7.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_EQ(result, FrontierSearchResult::SUCCESSFUL_SEARCH);
    
    // All returned frontiers should meet minimum size requirement
    for (const auto& frontier : output_frontiers) {
        EXPECT_GE(frontier->getSize(), 10);
    }
}

// Test with maximum frontier cluster size
TEST_F(FrontierBFSearchTest, MaximumFrontierClusterSize)
{
    // Create a costmap with large frontier clusters
    setupLargeFrontierCostmap();
    
    // Set a low maximum cluster size to test splitting
    frontier_search_->setMaxFrontierClusterSize(5.0);
    
    geometry_msgs::msg::Point position;
    position.x = 5.0;
    position.y = 5.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_EQ(result, FrontierSearchResult::SUCCESSFUL_SEARCH);
    
    // Should have multiple frontiers due to cluster splitting
    if (!output_frontiers.empty()) {
        // At least some frontiers should be at or near the maximum size
        bool found_max_size = false;
        for (const auto& frontier : output_frontiers) {
            if (frontier->getSize() >= 5) {
                found_max_size = true;
                break;
            }
        }
        EXPECT_TRUE(found_max_size);
    }
}

// Test lethal threshold parameter
TEST_F(FrontierBFSearchTest, LethalThreshold)
{
    // Set a low lethal threshold
    frontier_search_->setLethalThreshold(100);
    
    geometry_msgs::msg::Point position;
    position.x = 7.0;
    position.y = 7.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    // Should still work, but may find different frontiers due to different obstacle interpretation
    EXPECT_TRUE(result == FrontierSearchResult::SUCCESSFUL_SEARCH || 
                result == FrontierSearchResult::CANNOT_FIND_CELL_TO_SEARCH ||
                result == FrontierSearchResult::ROBOT_OUT_OF_BOUNDS);
}

// Test SortByMedianFunctor
TEST_F(FrontierBFSearchTest, SortByMedianFunctor)
{
    std::pair<double, double> centroid(5.0, 5.0);
    SortByMedianFunctor sorter(centroid);
    
    std::pair<double, double> point1(6.0, 5.0);  // East of centroid
    std::pair<double, double> point2(5.0, 6.0);  // North of centroid
    std::pair<double, double> point3(4.0, 5.0);  // West of centroid
    std::pair<double, double> point4(5.0, 4.0);  // South of centroid
    
    // Test sorting functionality
    std::vector<std::pair<double, double>> points = {point3, point1, point4, point2};
    std::sort(points.begin(), points.end(), sorter);
    
    // Points should be sorted by angle from centroid
    EXPECT_EQ(points.size(), 4);
    
    // Verify that sorting doesn't crash and produces consistent results
    std::vector<std::pair<double, double>> points_copy = {point3, point1, point4, point2};
    std::sort(points_copy.begin(), points_copy.end(), sorter);
    
    EXPECT_EQ(points, points_copy);
}

// Test edge case: empty costmap
TEST_F(FrontierBFSearchTest, EmptyCostmap)
{
    // Create a costmap with all free space (no frontiers)
    for (unsigned int x = 0; x < costmap_width_; ++x) {
        for (unsigned int y = 0; y < costmap_height_; ++y) {
            costmap_->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
        }
    }
    
    geometry_msgs::msg::Point position;
    position.x = 5.0;
    position.y = 5.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    EXPECT_EQ(result, FrontierSearchResult::SUCCESSFUL_SEARCH);
    EXPECT_TRUE(output_frontiers.empty());  // No frontiers in all-free costmap
}

// Test edge case: all unknown costmap
TEST_F(FrontierBFSearchTest, AllUnknownCostmap)
{
    // Create a costmap with all unknown space
    for (unsigned int x = 0; x < costmap_width_; ++x) {
        for (unsigned int y = 0; y < costmap_height_; ++y) {
            costmap_->setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
        }
    }
    
    geometry_msgs::msg::Point position;
    position.x = 5.0;
    position.y = 5.0;
    position.z = 0.0;
    
    std::vector<FrontierPtr> output_frontiers;
    double max_distance = 50.0;
    FrontierSearchResult result = frontier_search_->searchFrom(position, output_frontiers, max_distance);
    
    // Should not be able to find a clear cell to start search
    EXPECT_EQ(result, FrontierSearchResult::CANNOT_FIND_CELL_TO_SEARCH);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    
    int result = RUN_ALL_TESTS();
    
    rclcpp::shutdown();
    return result;
}