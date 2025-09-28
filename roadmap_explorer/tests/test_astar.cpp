#include <gtest/gtest.h>
#include <memory>
#include <unordered_map>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "roadmap_explorer/planners/Astar.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"

// Test environment class to manage ROS initialization
class AStarTestEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    // Initialize ROS if not already initialized
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  void TearDown() override
  {
    // Don't shutdown ROS here as it might be used by other tests
    // rclcpp::shutdown() will be called in main()
  }
};

// Test-friendly derived class that exposes protected methods
class TestableFrontierRoadmapAStar : public FrontierRoadmapAStar
{
public:
  using FrontierRoadmapAStar::heuristic;
  using FrontierRoadmapAStar::getSuccessors;
};

class AStarTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    astar_ = std::make_unique<FrontierRoadmapAStar>();
    testable_astar_ = std::make_unique<TestableFrontierRoadmapAStar>();
    
    // Create test frontiers
    frontier1_ = std::make_shared<Frontier>();
    frontier1_->setGoalPoint(0.0, 0.0);
    frontier1_->setUID(1);
    
    frontier2_ = std::make_shared<Frontier>();
    frontier2_->setGoalPoint(1.0, 0.0);
    frontier2_->setUID(2);
    
    frontier3_ = std::make_shared<Frontier>();
    frontier3_->setGoalPoint(0.0, 1.0);
    frontier3_->setUID(3);
    
    frontier4_ = std::make_shared<Frontier>();
    frontier4_->setGoalPoint(1.0, 1.0);
    frontier4_->setUID(4);
    
    frontier5_ = std::make_shared<Frontier>();
    frontier5_->setGoalPoint(2.0, 2.0);
    frontier5_->setUID(5);
  }

  void TearDown() override
  {
    astar_.reset();
    testable_astar_.reset();
  }

  // Helper function to create a simple linear roadmap
  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> createLinearRoadmap()
  {
    std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> roadmap;
    roadmap[frontier1_] = {frontier2_};
    roadmap[frontier2_] = {frontier1_, frontier3_};
    roadmap[frontier3_] = {frontier2_, frontier4_};
    roadmap[frontier4_] = {frontier3_};
    return roadmap;
  }

  // Helper function to create a grid-like roadmap
  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> createGridRoadmap()
  {
    std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> roadmap;
    roadmap[frontier1_] = {frontier2_, frontier3_};  // (0,0) -> (1,0), (0,1)
    roadmap[frontier2_] = {frontier1_, frontier4_};  // (1,0) -> (0,0), (1,1)
    roadmap[frontier3_] = {frontier1_, frontier4_};  // (0,1) -> (0,0), (1,1)
    roadmap[frontier4_] = {frontier2_, frontier3_};  // (1,1) -> (1,0), (0,1)
    return roadmap;
  }

  // Helper function to create a disconnected roadmap
  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> createDisconnectedRoadmap()
  {
    std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> roadmap;
    roadmap[frontier1_] = {frontier2_};  // Connected component 1
    roadmap[frontier2_] = {frontier1_};
    roadmap[frontier3_] = {frontier4_};  // Connected component 2
    roadmap[frontier4_] = {frontier3_};
    return roadmap;
  }

  std::unique_ptr<FrontierRoadmapAStar> astar_;
  std::unique_ptr<TestableFrontierRoadmapAStar> testable_astar_;
  FrontierPtr frontier1_, frontier2_, frontier3_, frontier4_, frontier5_;
};

// Test Node structure
TEST_F(AStarTest, NodeConstructorAndOperators)
{
  // Test default constructor
  Node node1;
  EXPECT_EQ(node1.g, 0.0);
  EXPECT_EQ(node1.h, 0.0);
  EXPECT_EQ(node1.f, 0.0);
  EXPECT_EQ(node1.parent, nullptr);
  EXPECT_EQ(node1.frontier, nullptr);

  // Test parameterized constructor
  Node node2(frontier1_, 5.0, 3.0);
  EXPECT_EQ(node2.frontier, frontier1_);
  EXPECT_EQ(node2.g, 5.0);
  EXPECT_EQ(node2.h, 3.0);
  EXPECT_EQ(node2.f, 8.0);  // f = g + h
  EXPECT_EQ(node2.parent, nullptr);

  // Test with parent
  auto parent_node = std::make_shared<Node>(frontier2_, 2.0, 1.0);
  Node node3(frontier3_, 7.0, 4.0, parent_node);
  EXPECT_EQ(node3.frontier, frontier3_);
  EXPECT_EQ(node3.g, 7.0);
  EXPECT_EQ(node3.h, 4.0);
  EXPECT_EQ(node3.f, 11.0);
  EXPECT_EQ(node3.parent, parent_node);

  // Test comparison operator
  Node node4(frontier4_, 10.0, 2.0);  // f = 12.0
  EXPECT_TRUE(node4 > node3);  // 12.0 > 11.0
  EXPECT_FALSE(node3 > node4);
}

// Test FCostNodeCompare
TEST_F(AStarTest, FCostNodeCompare)
{
  auto node1 = std::make_shared<Node>(frontier1_, 5.0, 3.0);  // f = 8.0
  auto node2 = std::make_shared<Node>(frontier2_, 7.0, 4.0);  // f = 11.0
  auto node3 = std::make_shared<Node>(frontier3_, 2.0, 6.0);  // f = 8.0

  FCostNodeCompare comparator;
  
  EXPECT_TRUE(comparator(node2, node1));   // 11.0 > 8.0
  EXPECT_FALSE(comparator(node1, node2));  // 8.0 < 11.0
  
  // Test tie-breaking: when f-costs are equal, prefer lower h-cost
  EXPECT_TRUE(comparator(node3, node1));   // Same f-cost (8.0), but node3.h (6.0) > node1.h (3.0)
  EXPECT_FALSE(comparator(node1, node3));  // Same f-cost (8.0), but node1.h (3.0) < node3.h (6.0)
}

// Test heuristic functions
TEST_F(AStarTest, HeuristicFunctions)
{
  Node node1(frontier1_, 0.0, 0.0);  // (0,0)
  Node node2(frontier2_, 0.0, 0.0);  // (1,0)
  Node node3(frontier4_, 0.0, 0.0);  // (1,1)

  // Test heuristic with Node references
  double h1 = testable_astar_->heuristic(node1, node2);
  EXPECT_DOUBLE_EQ(h1, 1.0);  // sqrt((1-0)^2 + (0-0)^2) = 1, but we return squared distance

  double h2 = testable_astar_->heuristic(node1, node3);
  EXPECT_DOUBLE_EQ(h2, 2.0);  // (1-0)^2 + (1-0)^2 = 2

  // Test heuristic with shared_ptr<Node>
  auto node_ptr1 = std::make_shared<Node>(frontier1_, 0.0, 0.0);
  auto node_ptr2 = std::make_shared<Node>(frontier2_, 0.0, 0.0);
  
  double h3 = testable_astar_->heuristic(node_ptr1, node_ptr2);
  EXPECT_DOUBLE_EQ(h3, 1.0);

  // Test heuristic with FrontierPtr
  double h4 = testable_astar_->heuristic(frontier1_, frontier2_);
  EXPECT_DOUBLE_EQ(h4, 1.0);

  double h5 = testable_astar_->heuristic(frontier1_, frontier4_);
  EXPECT_DOUBLE_EQ(h5, 2.0);
}

// Test getSuccessors method
TEST_F(AStarTest, GetSuccessors)
{
  auto roadmap = createGridRoadmap();
  
  auto current_node = std::make_shared<Node>(frontier1_, 5.0, 0.0);
  auto goal_node = std::make_shared<Node>(frontier4_, 0.0, 0.0);
  
  std::vector<std::shared_ptr<Node>> successors = testable_astar_->getSuccessors(current_node, goal_node, roadmap);
  
  EXPECT_EQ(successors.size(), 2);  // frontier1 connects to frontier2 and frontier3
  
  // Check that successors have correct frontiers
  std::vector<FrontierPtr> successor_frontiers;
  for (const auto& successor : successors) {
    successor_frontiers.push_back(successor->frontier);
  }
  
  EXPECT_TRUE(std::find(successor_frontiers.begin(), successor_frontiers.end(), frontier2_) != successor_frontiers.end());
  EXPECT_TRUE(std::find(successor_frontiers.begin(), successor_frontiers.end(), frontier3_) != successor_frontiers.end());
  
  // Check that g costs are calculated correctly
  for (const auto& successor : successors) {
    double expected_g = current_node->g + roadmap_explorer::sqDistanceBetweenFrontiers(current_node->frontier, successor->frontier);
    EXPECT_DOUBLE_EQ(successor->g, expected_g);
  }
}

// Test getSuccessors with frontier not in roadmap (should throw exception)
TEST_F(AStarTest, GetSuccessorsThrowsOnMissingFrontier)
{
  auto roadmap = createGridRoadmap();
  
  auto current_node = std::make_shared<Node>(frontier5_, 5.0, 0.0);  // frontier5 not in roadmap
  auto goal_node = std::make_shared<Node>(frontier4_, 0.0, 0.0);
  
  EXPECT_THROW(testable_astar_->getSuccessors(current_node, goal_node, roadmap), RoadmapExplorerException);
}

// Test A* algorithm - simple path
TEST_F(AStarTest, GetPlanSimplePath)
{
  auto roadmap = createLinearRoadmap();
  
  auto result = astar_->getPlan(frontier1_, frontier4_, roadmap);
  auto path = result.first;
  auto path_length = result.second;
  
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), 4);  // frontier1 -> frontier2 -> frontier3 -> frontier4
  
  // Check path order
  if (!path.empty()) {
    EXPECT_EQ(path[0]->frontier, frontier1_);
    if (path.size() > 1) {
      EXPECT_EQ(path[1]->frontier, frontier2_);
    }
    if (path.size() > 2) {
      EXPECT_EQ(path[2]->frontier, frontier3_);
    }
    if (path.size() > 3) {
      EXPECT_EQ(path[3]->frontier, frontier4_);
    }
  }
  
  // Check that path length is reasonable
  EXPECT_GT(path_length, 0.0);
}

// Test A* algorithm - same start and goal
TEST_F(AStarTest, GetPlanSameStartAndGoal)
{
  auto roadmap = createGridRoadmap();
  
  auto result = astar_->getPlan(frontier1_, frontier1_, roadmap);
  auto path = result.first;
  auto path_length = result.second;
  
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), 1);
  EXPECT_EQ(path[0]->frontier, frontier1_);
  EXPECT_DOUBLE_EQ(path_length, 0.0);
}

// Test A* algorithm - no path exists
TEST_F(AStarTest, GetPlanNoPath)
{
  auto roadmap = createDisconnectedRoadmap();
  
  // Try to find path from component 1 to component 2
  auto result = astar_->getPlan(frontier1_, frontier3_, roadmap);
  auto path = result.first;
  auto path_length = result.second;
  
  EXPECT_TRUE(path.empty());
  EXPECT_DOUBLE_EQ(path_length, 0.0);
}

// Test A* algorithm - grid pathfinding
TEST_F(AStarTest, GetPlanGridPath)
{
  auto roadmap = createGridRoadmap();
  
  auto result = astar_->getPlan(frontier1_, frontier4_, roadmap);
  auto path = result.first;
  auto path_length = result.second;
  
  EXPECT_FALSE(path.empty());
  EXPECT_GE(path.size(), 2);  // At least start and goal
  
  // Check that path starts and ends correctly
  EXPECT_EQ(path[0]->frontier, frontier1_);
  EXPECT_EQ(path[path.size() - 1]->frontier, frontier4_);
  
  // Path length should be reasonable (diagonal distance is sqrt(2))
  EXPECT_GT(path_length, 0.0);
  EXPECT_LE(path_length, 3.0);  // Should not be much longer than optimal
}

// Test A* algorithm with empty roadmap
TEST_F(AStarTest, GetPlanEmptyRoadmap)
{
  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> empty_roadmap;
  
  EXPECT_THROW(astar_->getPlan(frontier1_, frontier2_, empty_roadmap), RoadmapExplorerException);
}

// Test A* algorithm - complex roadmap with multiple paths
TEST_F(AStarTest, GetPlanComplexRoadmap)
{
  // Create a more complex roadmap with multiple possible paths
  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> roadmap;
  
  // Create additional frontier for more complex topology
  auto frontier6 = std::make_shared<Frontier>();
  frontier6->setGoalPoint(0.5, 0.5);
  frontier6->setUID(6);
  
  // Create a roadmap where there are multiple paths from start to goal
  roadmap[frontier1_] = {frontier2_, frontier3_, frontier6};
  roadmap[frontier2_] = {frontier1_, frontier4_, frontier6};
  roadmap[frontier3_] = {frontier1_, frontier4_, frontier6};
  roadmap[frontier4_] = {frontier2_, frontier3_, frontier6};
  roadmap[frontier6] = {frontier1_, frontier2_, frontier3_, frontier4_};
  
  auto result = astar_->getPlan(frontier1_, frontier4_, roadmap);
  auto path = result.first;
  auto path_length = result.second;
  
  EXPECT_FALSE(path.empty());
  EXPECT_GE(path.size(), 2);
  
  // Check that path starts and ends correctly
  EXPECT_EQ(path[0]->frontier, frontier1_);
  EXPECT_EQ(path[path.size() - 1]->frontier, frontier4_);
  
  // Use path_length to avoid unused variable warning
  EXPECT_GT(path_length, 0.0);
  
  // Verify path continuity
  for (size_t i = 1; i < path.size(); ++i) {
    auto prev_frontier = path[i-1]->frontier;
    auto curr_frontier = path[i]->frontier;
    
    // Check that current frontier is in the adjacency list of previous frontier
    auto& neighbors = roadmap[prev_frontier];
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), curr_frontier) != neighbors.end());
  }
}

// Performance test - larger roadmap
TEST_F(AStarTest, GetPlanPerformanceTest)
{
  // Create a larger roadmap for performance testing
  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> large_roadmap;
  std::vector<FrontierPtr> frontiers;
  
  // Create a 5x5 grid of frontiers
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      auto frontier = std::make_shared<Frontier>();
      frontier->setGoalPoint(i * 1.0, j * 1.0);
      frontier->setUID(i * 5 + j + 10);  // Unique IDs starting from 10
      frontiers.push_back(frontier);
    }
  }
  
  // Connect each frontier to its 4-connected neighbors
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      int idx = i * 5 + j;
      std::vector<FrontierPtr> neighbors;
      
      if (i > 0) neighbors.push_back(frontiers[(i-1) * 5 + j]);  // Up
      if (i < 4) neighbors.push_back(frontiers[(i+1) * 5 + j]);  // Down
      if (j > 0) neighbors.push_back(frontiers[i * 5 + (j-1)]);  // Left
      if (j < 4) neighbors.push_back(frontiers[i * 5 + (j+1)]);  // Right
      
      large_roadmap[frontiers[idx]] = neighbors;
    }
  }
  
  // Test pathfinding from corner to corner
  auto start_time = std::chrono::high_resolution_clock::now();
  auto result = astar_->getPlan(frontiers[0], frontiers[24], large_roadmap);  // (0,0) to (4,4)
  auto end_time = std::chrono::high_resolution_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  auto path = result.first;
  auto path_length = result.second;
  
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path[0]->frontier, frontiers[0]);
  EXPECT_EQ(path[path.size() - 1]->frontier, frontiers[24]);
  
  // Should find optimal path length (Manhattan distance = 8 steps)
  EXPECT_EQ(path.size(), 9);  // 8 steps + start node
  
  // Use path_length to avoid unused variable warning
  EXPECT_GT(path_length, 0.0);
  
  // Performance should be reasonable (less than 100ms for this small grid)
  EXPECT_LT(duration.count(), 100);
  
  std::cout << "A* pathfinding took " << duration.count() << " ms for 5x5 grid" << std::endl;
}

// Test edge case - single node roadmap
TEST_F(AStarTest, GetPlanSingleNodeRoadmap)
{
  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> roadmap;
  roadmap[frontier1_] = {};  // No connections
  
  // Same start and goal should work
  auto result = astar_->getPlan(frontier1_, frontier1_, roadmap);
  auto path = result.first;
  
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path.size(), 1);
  EXPECT_EQ(path[0]->frontier, frontier1_);
  
  // Different start and goal should fail
  auto result2 = astar_->getPlan(frontier1_, frontier2_, roadmap);
  auto path2 = result2.first;
  
  EXPECT_TRUE(path2.empty());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  
  // Register the test environment
  ::testing::AddGlobalTestEnvironment(new AStarTestEnvironment);
  
  int result = RUN_ALL_TESTS();
  
  // Shutdown ROS if it was initialized
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  
  return result;
}
