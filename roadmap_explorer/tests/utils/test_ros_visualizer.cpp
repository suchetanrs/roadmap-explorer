#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "roadmap_explorer/util/RosVisualizer.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/Logger.hpp"

class RosVisualizerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        
        // Create a lifecycle node for testing
        node_ = std::make_shared<nav2_util::LifecycleNode>("test_ros_visualizer");
        
        // Create a simple costmap for testing
        costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
            10, 10,  // width, height
            0.1,     // resolution
            0.0, 0.0 // origin_x, origin_y
        );
        
        // Initialize the costmap with some data
        for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); ++i) {
            for (unsigned int j = 0; j < costmap_->getSizeInCellsY(); ++j) {
                costmap_->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
            }
        }
        
        // Create test frontiers
        createTestFrontiers();
        
        // Create test points
        createTestPoints();
        
        // Create test poses
        createTestPoses();
        
        // Create test path
        createTestPath();
    }
    
    void TearDown() override
    {
        // Destroy RosVisualizer instance if it exists
        try {
            RosVisualizer::destroyInstance();
        } catch (const std::exception&) {
            // Instance might not exist, which is fine
        }
        
        rclcpp::shutdown();
    }
    
    void createTestFrontiers()
    {
        // Create test frontiers with different properties
        auto frontier1 = std::make_shared<Frontier>();
        geometry_msgs::msg::Point goal1;
        goal1.x = 1.0;
        goal1.y = 2.0;
        goal1.z = 0.0;
        frontier1->setGoalPoint(goal1);
        
        frontier1->setGoalOrientation(0.0); // theta = 0.0 radians
        
        frontier1->setSize(10);
        frontier1->setPathLength(5.0);
        frontier1->setArrivalInformation(0.8);
        frontier1->setAchievability(true);
        
        auto frontier2 = std::make_shared<Frontier>();
        geometry_msgs::msg::Point goal2;
        goal2.x = 3.0;
        goal2.y = 4.0;
        goal2.z = 0.0;
        frontier2->setGoalPoint(goal2);
        frontier2->setGoalOrientation(0.0); // theta = 0.0 radians
        frontier2->setSize(15);
        frontier2->setPathLength(7.5);
        frontier2->setArrivalInformation(0.6);
        frontier2->setAchievability(true);
        
        test_frontiers_ = {frontier1, frontier2};
        
        // Create blacklisted frontier
        auto blacklisted_frontier = std::make_shared<Frontier>();
        geometry_msgs::msg::Point goal3;
        goal3.x = 5.0;
        goal3.y = 6.0;
        goal3.z = 0.0;
        blacklisted_frontier->setGoalPoint(goal3);
        blacklisted_frontier->setGoalOrientation(0.0); // theta = 0.0 radians
        blacklisted_frontier->setSize(8);
        blacklisted_frontier->setAchievability(false);
        
        blacklisted_frontiers_ = {blacklisted_frontier};
    }
    
    void createTestPoints()
    {
        geometry_msgs::msg::Point p1, p2, p3;
        p1.x = 1.0; p1.y = 1.0; p1.z = 0.0;
        p2.x = 2.0; p2.y = 2.0; p2.z = 0.0;
        p3.x = 3.0; p3.y = 3.0; p3.z = 0.0;
        
        test_geometry_points_ = {p1, p2, p3};
        
        // Create map locations
        nav2_costmap_2d::MapLocation loc1, loc2, loc3;
        loc1.x = 1; loc1.y = 1;
        loc2.x = 2; loc2.y = 2;
        loc3.x = 3; loc3.y = 3;
        
        test_map_locations_ = {loc1, loc2, loc3};
        
        // Create double vector points
        test_double_points_ = {{1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0, 1.0}};
    }
    
    void createTestPoses()
    {
        geometry_msgs::msg::Pose pose1, pose2;
        pose1.position.x = 1.0;
        pose1.position.y = 1.0;
        pose1.position.z = 0.0;
        pose1.orientation.w = 1.0;
        
        pose2.position.x = 2.0;
        pose2.position.y = 2.0;
        pose2.position.z = 0.0;
        pose2.orientation.w = 1.0;
        
        test_poses_ = {pose1, pose2};
    }
    
    void createTestPath()
    {
        test_path_.header.frame_id = "map";
        test_path_.header.stamp = rclcpp::Clock().now();
        
        geometry_msgs::msg::PoseStamped pose1, pose2;
        pose1.header = test_path_.header;
        pose1.pose.position.x = 0.0;
        pose1.pose.position.y = 0.0;
        pose1.pose.orientation.w = 1.0;
        
        pose2.header = test_path_.header;
        pose2.pose.position.x = 1.0;
        pose2.pose.position.y = 1.0;
        pose2.pose.orientation.w = 1.0;
        
        test_path_.poses = {pose1, pose2};
    }
    
    std::shared_ptr<nav2_util::LifecycleNode> node_;
    std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    std::vector<FrontierPtr> test_frontiers_;
    std::vector<FrontierPtr> blacklisted_frontiers_;
    std::vector<geometry_msgs::msg::Point> test_geometry_points_;
    std::vector<nav2_costmap_2d::MapLocation> test_map_locations_;
    std::vector<std::vector<double>> test_double_points_;
    std::deque<geometry_msgs::msg::Pose> test_poses_;
    nav_msgs::msg::Path test_path_;
};

// Test singleton pattern
TEST_F(RosVisualizerTest, SingletonPattern)
{
    // Test that instance doesn't exist initially
    EXPECT_THROW(RosVisualizer::getInstance(), RoadmapExplorerException);
    
    // Create instance
    RosVisualizer::createInstance(node_, costmap_.get());
    
    // Test that we can get the instance
    EXPECT_NO_THROW(RosVisualizer::getInstance());
    
    // Test that creating another instance throws
    EXPECT_THROW(RosVisualizer::createInstance(node_, costmap_.get()), RoadmapExplorerException);
    
    // Test destroy
    RosVisualizer::destroyInstance();
    
    // Test that instance doesn't exist after destroy
    EXPECT_THROW(RosVisualizer::getInstance(), RoadmapExplorerException);
}

// Test constructor and destructor
TEST_F(RosVisualizerTest, ConstructorDestructor)
{
    // Create instance
    RosVisualizer::createInstance(node_, costmap_.get());
    
    // Verify instance exists and can be accessed
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test that we can call methods without crashing
    EXPECT_NO_THROW(visualizer.getNumSubscribers("test_topic"));
    
    // Destroy instance
    RosVisualizer::destroyInstance();
}

// Test visualizePointCloud with frontiers
TEST_F(RosVisualizerTest, VisualizePointCloudFromFrontiers)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with valid frontiers
    EXPECT_NO_THROW(visualizer.visualizePointCloud("test_frontiers", test_frontiers_, "map", 100.0f));
    
    // Test with empty frontiers
    std::vector<FrontierPtr> empty_frontiers;
    EXPECT_NO_THROW(visualizer.visualizePointCloud("test_empty_frontiers", empty_frontiers, "map", 50.0f));
    
    // Test with different frame_id
    EXPECT_NO_THROW(visualizer.visualizePointCloud("test_frontiers_odom", test_frontiers_, "odom", 75.0f));
    
    RosVisualizer::destroyInstance();
}

// Test visualizePointCloud with double points
TEST_F(RosVisualizerTest, VisualizePointCloudFromDoublePoints)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with valid points
    EXPECT_NO_THROW(visualizer.visualizePointCloud("test_double_points", test_double_points_, "map", 200.0f));
    
    // Test with empty points
    std::vector<std::vector<double>> empty_points;
    EXPECT_NO_THROW(visualizer.visualizePointCloud("test_empty_double_points", empty_points, "map", 150.0f));
    
    // Test with points of different dimensions
    std::vector<std::vector<double>> mixed_points = {{1.0}, {2.0, 2.0}, {3.0, 3.0, 3.0}};
    EXPECT_NO_THROW(visualizer.visualizePointCloud("test_mixed_points", mixed_points, "base_link", 300.0f));
    
    RosVisualizer::destroyInstance();
}

// Test visualizeMarkers with geometry points
TEST_F(RosVisualizerTest, VisualizeMarkersFromGeometryPoints)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with valid geometry points
    EXPECT_NO_THROW(visualizer.visualizeMarkers("test_geometry_markers", test_geometry_points_, "map"));
    
    // Test with empty points
    std::vector<geometry_msgs::msg::Point> empty_points;
    EXPECT_NO_THROW(visualizer.visualizeMarkers("test_empty_geometry_markers", empty_points, "map"));
    
    // Test with different frame_id
    EXPECT_NO_THROW(visualizer.visualizeMarkers("test_geometry_markers_odom", test_geometry_points_, "odom"));
    
    RosVisualizer::destroyInstance();
}

// Test visualizeMarkers with map locations
TEST_F(RosVisualizerTest, VisualizeMarkersFromMapLocations)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with valid map locations
    EXPECT_NO_THROW(visualizer.visualizeMarkers("test_map_markers", test_map_locations_, "map"));
    
    // Test with empty locations
    std::vector<nav2_costmap_2d::MapLocation> empty_locations;
    EXPECT_NO_THROW(visualizer.visualizeMarkers("test_empty_map_markers", empty_locations, "map"));
    
    // Test with different frame_id
    EXPECT_NO_THROW(visualizer.visualizeMarkers("test_map_markers_base", test_map_locations_, "base_link"));
    
    RosVisualizer::destroyInstance();
}

// Test visualizeFrontierMarkers
TEST_F(RosVisualizerTest, VisualizeFrontierMarkers)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with valid frontiers
    EXPECT_NO_THROW(visualizer.visualizeFrontierMarkers("test_frontier_markers", test_frontiers_, "map"));
    
    // Test with empty frontiers
    std::vector<FrontierPtr> empty_frontiers;
    EXPECT_NO_THROW(visualizer.visualizeFrontierMarkers("test_empty_frontier_markers", empty_frontiers, "map"));
    
    // Test with mix of achievable and non-achievable frontiers
    auto mixed_frontiers = test_frontiers_;
    mixed_frontiers.insert(mixed_frontiers.end(), blacklisted_frontiers_.begin(), blacklisted_frontiers_.end());
    EXPECT_NO_THROW(visualizer.visualizeFrontierMarkers("test_mixed_frontier_markers", mixed_frontiers, "odom"));
    
    RosVisualizer::destroyInstance();
}

// Test visualizeBlacklistedFrontierMarkers
TEST_F(RosVisualizerTest, VisualizeBlacklistedFrontierMarkers)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with blacklisted frontiers
    EXPECT_NO_THROW(visualizer.visualizeBlacklistedFrontierMarkers("test_blacklisted_markers", blacklisted_frontiers_, "map"));
    
    // Test with empty blacklisted frontiers
    std::vector<FrontierPtr> empty_frontiers;
    EXPECT_NO_THROW(visualizer.visualizeBlacklistedFrontierMarkers("test_empty_blacklisted_markers", empty_frontiers, "map"));
    
    // Test with different frame_id
    EXPECT_NO_THROW(visualizer.visualizeBlacklistedFrontierMarkers("test_blacklisted_markers_odom", blacklisted_frontiers_, "odom"));
    
    RosVisualizer::destroyInstance();
}

// Test visualizePath
TEST_F(RosVisualizerTest, VisualizePath)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with valid path
    EXPECT_NO_THROW(visualizer.visualizePath("test_path", test_path_));
    
    // Test with empty path
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = "map";
    empty_path.header.stamp = rclcpp::Clock().now();
    EXPECT_NO_THROW(visualizer.visualizePath("test_empty_path", empty_path));
    
    RosVisualizer::destroyInstance();
}

// Test visualizePoseArray
TEST_F(RosVisualizerTest, VisualizePoseArray)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with valid poses
    EXPECT_NO_THROW(visualizer.visualizePoseArray("test_pose_array", test_poses_, "map"));
    
    // Test with empty poses
    std::deque<geometry_msgs::msg::Pose> empty_poses;
    EXPECT_NO_THROW(visualizer.visualizePoseArray("test_empty_pose_array", empty_poses, "map"));
    
    // Test with different frame_id
    EXPECT_NO_THROW(visualizer.visualizePoseArray("test_pose_array_odom", test_poses_, "odom"));
    
    RosVisualizer::destroyInstance();
}

// Test getNumSubscribers
TEST_F(RosVisualizerTest, GetNumSubscribers)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test with non-existent topic
    EXPECT_EQ(visualizer.getNumSubscribers("non_existent_topic"), 0);
    
    // Test with pre-created topics (should exist but have 0 subscribers)
    EXPECT_EQ(visualizer.getNumSubscribers("frontiers"), 0);
    EXPECT_EQ(visualizer.getNumSubscribers("observable_cells"), 0);
    EXPECT_EQ(visualizer.getNumSubscribers("full_path"), 0);
    
    // Create a publisher and test
    visualizer.visualizePointCloud("test_subscriber_count", test_frontiers_, "map");
    EXPECT_EQ(visualizer.getNumSubscribers("test_subscriber_count"), 0); // No actual subscribers
    
    RosVisualizer::destroyInstance();
}

// Test error conditions
TEST_F(RosVisualizerTest, ErrorConditions)
{
    // Test calling methods without costmap (should throw for marker methods)
    auto node_no_costmap = std::make_shared<nav2_util::LifecycleNode>("test_no_costmap");
    RosVisualizer::createInstance(node_no_costmap, nullptr);
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // These should throw because costmap is nullptr
    EXPECT_THROW(visualizer.visualizeMarkers("test_error", test_geometry_points_, "map"), RoadmapExplorerException);
    EXPECT_THROW(visualizer.visualizeMarkers("test_error", test_map_locations_, "map"), RoadmapExplorerException);
    
    // These should not throw (don't require costmap)
    EXPECT_NO_THROW(visualizer.visualizePointCloud("test_no_error", test_frontiers_, "map"));
    EXPECT_NO_THROW(visualizer.visualizeFrontierMarkers("test_no_error", test_frontiers_, "map"));
    EXPECT_NO_THROW(visualizer.visualizePath("test_no_error", test_path_));
    EXPECT_NO_THROW(visualizer.visualizePoseArray("test_no_error", test_poses_, "map"));
    
    RosVisualizer::destroyInstance();
}

// Test thread safety
TEST_F(RosVisualizerTest, ThreadSafety)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Create multiple threads that access the visualizer simultaneously
    std::vector<std::thread> threads;
    const int num_threads = 5;
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&visualizer, i]() {
            std::string topic_name = "thread_test_" + std::to_string(i);
            
            // Each thread performs different visualization operations
            switch (i % 4) {
                case 0: {
                    std::vector<FrontierPtr> empty_frontiers;
                    visualizer.visualizePointCloud(topic_name + "_pc", empty_frontiers, "map");
                    break;
                }
                case 1: {
                    std::vector<geometry_msgs::msg::Point> empty_points;
                    visualizer.visualizeMarkers(topic_name + "_markers", empty_points, "map");
                    break;
                }
                case 2: {
                    std::vector<FrontierPtr> empty_frontiers;
                    visualizer.visualizeFrontierMarkers(topic_name + "_frontiers", empty_frontiers, "map");
                    break;
                }
                case 3: {
                    nav_msgs::msg::Path empty_path;
                    empty_path.header.frame_id = "map";
                    visualizer.visualizePath(topic_name + "_path", empty_path);
                    break;
                }
            }
            
            // Test getNumSubscribers
            visualizer.getNumSubscribers(topic_name);
        });
    }
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
    
    // If we reach here without crashing, thread safety test passed
    SUCCEED();
    
    RosVisualizer::destroyInstance();
}

// Test publisher reuse
TEST_F(RosVisualizerTest, PublisherReuse)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    const std::string topic_name = "reuse_test_topic";
    
    // Use the same topic multiple times
    EXPECT_NO_THROW(visualizer.visualizePointCloud(topic_name, test_frontiers_, "map"));
    std::vector<FrontierPtr> empty_frontiers;
    EXPECT_NO_THROW(visualizer.visualizePointCloud(topic_name, empty_frontiers, "map"));
    EXPECT_NO_THROW(visualizer.visualizePointCloud(topic_name, test_frontiers_, "odom"));
    
    // Verify subscriber count is consistent
    size_t count1 = visualizer.getNumSubscribers(topic_name);
    size_t count2 = visualizer.getNumSubscribers(topic_name);
    EXPECT_EQ(count1, count2);
    
    RosVisualizer::destroyInstance();
}

// Test all pre-created legacy publishers
TEST_F(RosVisualizerTest, LegacyPublishers)
{
    RosVisualizer::createInstance(node_, costmap_.get());
    RosVisualizer& visualizer = RosVisualizer::getInstance();
    
    // Test that all legacy publishers are pre-created and accessible
    std::vector<std::string> legacy_topics = {
        "observable_cells",
        "connecting_cells",
        "spatial_hashmap_points",
        "frontiers",
        "all_frontiers",
        "frontier_cell_markers",
        "blacklisted_frontiers",
        "grid_based_frontier_plan",
        "full_path",
        "global_repositioning_path",
        "trailing_robot_poses"
    };
    
    for (const auto& topic : legacy_topics) {
        // Should return 0 (no subscribers) but not throw
        EXPECT_NO_THROW(visualizer.getNumSubscribers(topic));
        EXPECT_EQ(visualizer.getNumSubscribers(topic), 0);
    }
    
    RosVisualizer::destroyInstance();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
