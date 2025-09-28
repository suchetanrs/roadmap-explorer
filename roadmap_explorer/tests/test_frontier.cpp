#include <gtest/gtest.h>
#include <memory>
#include <sstream>
#include <unordered_set>

#include "roadmap_explorer/Frontier.hpp"

class FrontierTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    frontier_ = std::make_unique<Frontier>();
  }

  void TearDown() override
  {
    frontier_.reset();
  }

  std::unique_ptr<Frontier> frontier_;
};

// Test constructor and initial state
TEST_F(FrontierTest, ConstructorInitializesCorrectly)
{
  EXPECT_TRUE(frontier_->isFrontierNull());
  EXPECT_TRUE(frontier_->isAchievable());
  EXPECT_FALSE(frontier_->isBlacklisted());
}

// Test UID functionality
TEST_F(FrontierTest, UIDFunctionality)
{
  const size_t test_uid = 12345;
  frontier_->setUID(test_uid);
  EXPECT_EQ(frontier_->getUID(), test_uid);
}

TEST_F(FrontierTest, UIDThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getUID(), RoadmapExplorerException);
}

// Test Size functionality
TEST_F(FrontierTest, SizeFunctionality)
{
  const int test_size = 100;
  frontier_->setSize(test_size);
  EXPECT_EQ(frontier_->getSize(), test_size);
}

TEST_F(FrontierTest, SizeThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getSize(), RoadmapExplorerException);
}

// Test Goal Point functionality
TEST_F(FrontierTest, GoalPointFunctionality)
{
  geometry_msgs::msg::Point test_point;
  test_point.x = 1.5;
  test_point.y = 2.5;
  test_point.z = 0.0;

  frontier_->setGoalPoint(test_point);
  
  const auto& retrieved_point = frontier_->getGoalPoint();
  EXPECT_DOUBLE_EQ(retrieved_point.x, 1.5);
  EXPECT_DOUBLE_EQ(retrieved_point.y, 2.5);
  EXPECT_DOUBLE_EQ(retrieved_point.z, 0.0);
  
  EXPECT_FALSE(frontier_->isFrontierNull());
}

TEST_F(FrontierTest, GoalPointFunctionalityWithCoordinates)
{
  const double x = 3.14;
  const double y = 2.71;

  frontier_->setGoalPoint(x, y);
  
  const auto& retrieved_point = frontier_->getGoalPoint();
  EXPECT_DOUBLE_EQ(retrieved_point.x, x);
  EXPECT_DOUBLE_EQ(retrieved_point.y, y);
  EXPECT_DOUBLE_EQ(retrieved_point.z, 0.0);
}

TEST_F(FrontierTest, GoalPointThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getGoalPoint(), RoadmapExplorerException);
}

// Test Goal Orientation functionality
TEST_F(FrontierTest, GoalOrientationFunctionality)
{
  const double theta = M_PI / 4.0;  // 45 degrees
  frontier_->setGoalOrientation(theta);
  
  const auto& orientation = frontier_->getGoalOrientation();
  
  // Check that the orientation is properly set using nav2_util::geometry_utils
  // The exact values depend on the implementation of orientationAroundZAxis
  EXPECT_TRUE(std::abs(orientation.x) < 1e-6);  // Should be close to 0
  EXPECT_TRUE(std::abs(orientation.y) < 1e-6);  // Should be close to 0
  EXPECT_TRUE(orientation.w > 0);  // Should be positive for the given angle
}

TEST_F(FrontierTest, GoalOrientationThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getGoalOrientation(), RoadmapExplorerException);
}

// Test Arrival Information functionality
TEST_F(FrontierTest, ArrivalInformationFunctionality)
{
  const double info = 0.85;
  frontier_->setArrivalInformation(info);
  EXPECT_DOUBLE_EQ(frontier_->getArrivalInformation(), info);
}

TEST_F(FrontierTest, ArrivalInformationThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getArrivalInformation(), RoadmapExplorerException);
}

// Test Path Length functionality
TEST_F(FrontierTest, PathLengthFunctionality)
{
  const double path_length = 15.75;
  frontier_->setPathLength(path_length);
  EXPECT_DOUBLE_EQ(frontier_->getPathLength(), path_length);
}

TEST_F(FrontierTest, PathLengthThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getPathLength(), RoadmapExplorerException);
}

// Test Path Length in Meters functionality
TEST_F(FrontierTest, PathLengthInMFunctionality)
{
  const double path_length_m = 12.34;
  frontier_->setPathLengthInM(path_length_m);
  EXPECT_DOUBLE_EQ(frontier_->getPathLengthInM(), path_length_m);
}

TEST_F(FrontierTest, PathLengthInMThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getPathLengthInM(), RoadmapExplorerException);
}

// Test Path Heading functionality
TEST_F(FrontierTest, PathHeadingFunctionality)
{
  const double heading = M_PI / 3.0;  // 60 degrees
  frontier_->setPathHeading(heading);
  EXPECT_DOUBLE_EQ(frontier_->getPathHeading(), heading);
}

TEST_F(FrontierTest, PathHeadingThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getPathHeading(), RoadmapExplorerException);
}

// Test Cost functionality
TEST_F(FrontierTest, CostFunctionality)
{
  const std::string cost_name = "distance";
  const double cost_value = 5.5;
  
  frontier_->setCost(cost_name, cost_value);
  EXPECT_DOUBLE_EQ(frontier_->getCost(cost_name), cost_value);
}

TEST_F(FrontierTest, MultipleCostsFunctionality)
{
  frontier_->setCost("distance", 10.0);
  frontier_->setCost("time", 20.0);
  frontier_->setCost("energy", 30.0);
  
  EXPECT_DOUBLE_EQ(frontier_->getCost("distance"), 10.0);
  EXPECT_DOUBLE_EQ(frontier_->getCost("time"), 20.0);
  EXPECT_DOUBLE_EQ(frontier_->getCost("energy"), 30.0);
}

TEST_F(FrontierTest, CostThrowsWhenMapNull)
{
  EXPECT_THROW(frontier_->getCost("nonexistent"), RoadmapExplorerException);
}

TEST_F(FrontierTest, CostThrowsWhenKeyNotFound)
{
  frontier_->setCost("existing", 1.0);
  EXPECT_THROW(frontier_->getCost("nonexistent"), RoadmapExplorerException);
}

// Test Weighted Cost functionality
TEST_F(FrontierTest, WeightedCostFunctionality)
{
  const double weighted_cost = 42.0;
  frontier_->setWeightedCost(weighted_cost);
  EXPECT_DOUBLE_EQ(frontier_->getWeightedCost(), weighted_cost);
}

TEST_F(FrontierTest, WeightedCostThrowsWhenNull)
{
  EXPECT_THROW(frontier_->getWeightedCost(), RoadmapExplorerException);
}

// Test Achievability functionality
TEST_F(FrontierTest, AchievabilityFunctionality)
{
  EXPECT_TRUE(frontier_->isAchievable());  // Default is true
  
  frontier_->setAchievability(false);
  EXPECT_FALSE(frontier_->isAchievable());
  
  frontier_->setAchievability(true);
  EXPECT_TRUE(frontier_->isAchievable());
}

// Test Blacklisted functionality
TEST_F(FrontierTest, BlacklistedFunctionality)
{
  EXPECT_FALSE(frontier_->isBlacklisted());  // Default is false
  
  frontier_->setBlacklisted(true);
  EXPECT_TRUE(frontier_->isBlacklisted());
  
  frontier_->setBlacklisted(false);
  EXPECT_FALSE(frontier_->isBlacklisted());
}

// Test Equality operator
TEST_F(FrontierTest, EqualityOperator)
{
  auto frontier2 = std::make_unique<Frontier>();
  
  geometry_msgs::msg::Point point1, point2;
  point1.x = 1.0; point1.y = 2.0; point1.z = 0.0;
  point2.x = 1.0; point2.y = 2.0; point2.z = 0.0;
  
  frontier_->setGoalPoint(point1);
  frontier2->setGoalPoint(point2);
  
  EXPECT_TRUE(*frontier_ == *frontier2);
  
  // Change one point
  point2.x = 3.0;
  frontier2->setGoalPoint(point2);
  EXPECT_FALSE(*frontier_ == *frontier2);
}

TEST_F(FrontierTest, EqualityOperatorThrowsWhenNull)
{
  auto frontier2 = std::make_unique<Frontier>();
  geometry_msgs::msg::Point point;
  point.x = 1.0; point.y = 2.0; point.z = 0.0;
  
  frontier_->setGoalPoint(point);
  // frontier2 has null goal_point
  
  EXPECT_THROW(*frontier_ == *frontier2, RoadmapExplorerException);
}

// Test Less-than operator
TEST_F(FrontierTest, LessThanOperator)
{
  auto frontier2 = std::make_unique<Frontier>();
  
  frontier_->setUID(100);
  frontier2->setUID(200);
  
  EXPECT_TRUE(*frontier_ < *frontier2);
  EXPECT_FALSE(*frontier2 < *frontier_);
}

// Test Stream operator
TEST_F(FrontierTest, StreamOperator)
{
  geometry_msgs::msg::Point point;
  point.x = 3.14;
  point.y = 2.71;
  point.z = 0.0;
  
  frontier_->setGoalPoint(point);
  
  std::ostringstream oss;
  oss << *frontier_;
  
  std::string output = oss.str();
  EXPECT_TRUE(output.find("Frontier(x: 3.14, y: 2.71)") != std::string::npos);
}

// Test FrontierPtr and utility functions
TEST_F(FrontierTest, FrontierPtrAndUtilities)
{
  auto frontier_ptr = std::make_shared<Frontier>();
  geometry_msgs::msg::Point point;
  point.x = 5.0;
  point.y = 10.0;
  point.z = 0.0;
  
  frontier_ptr->setGoalPoint(point);
  frontier_ptr->setUID(123);
  
  // Test generateUID function
  size_t generated_uid = generateUID(frontier_ptr);
  EXPECT_GT(generated_uid, 0);
  
  // Test FrontierHash
  FrontierHash hasher;
  size_t hash_value = hasher(frontier_ptr);
  EXPECT_GT(hash_value, 0);
  
  // Test FrontierGoalPointEquality
  auto frontier_ptr2 = std::make_shared<Frontier>();
  frontier_ptr2->setGoalPoint(point);
  
  FrontierGoalPointEquality equality_checker;
  EXPECT_TRUE(equality_checker(frontier_ptr, frontier_ptr2));
  
  // Change point in second frontier
  geometry_msgs::msg::Point different_point;
  different_point.x = 15.0;
  different_point.y = 20.0;
  different_point.z = 0.0;
  frontier_ptr2->setGoalPoint(different_point);
  
  EXPECT_FALSE(equality_checker(frontier_ptr, frontier_ptr2));
}

// Test setValue template function
TEST_F(FrontierTest, SetValueTemplateFunction)
{
  std::shared_ptr<int> int_ptr = nullptr;
  setValue(int_ptr, 42);
  EXPECT_EQ(*int_ptr, 42);
  
  // Test updating existing value
  setValue(int_ptr, 84);
  EXPECT_EQ(*int_ptr, 84);
}

// Test setMapValue function
TEST_F(FrontierTest, SetMapValueFunction)
{
  std::shared_ptr<std::map<std::string, double>> map_ptr = nullptr;
  
  setMapValue(map_ptr, "key1", 1.5);
  EXPECT_EQ((*map_ptr)["key1"], 1.5);
  
  setMapValue(map_ptr, "key2", 2.5);
  EXPECT_EQ((*map_ptr)["key1"], 1.5);
  EXPECT_EQ((*map_ptr)["key2"], 2.5);
  
  // Test updating existing key
  setMapValue(map_ptr, "key1", 3.5);
  EXPECT_EQ((*map_ptr)["key1"], 3.5);
  EXPECT_EQ((*map_ptr)["key2"], 2.5);
}

// Integration test - full frontier setup
TEST_F(FrontierTest, FullFrontierSetup)
{
  // Set up a complete frontier
  frontier_->setUID(999);
  frontier_->setSize(50);
  frontier_->setGoalPoint(10.0, 20.0);
  frontier_->setGoalOrientation(M_PI / 2.0);
  frontier_->setArrivalInformation(0.95);
  frontier_->setPathLength(25.5);
  frontier_->setPathLengthInM(25.5);
  frontier_->setPathHeading(M_PI / 6.0);
  frontier_->setCost("distance", 25.5);
  frontier_->setCost("time", 10.2);
  frontier_->setWeightedCost(35.7);
  frontier_->setAchievability(true);
  frontier_->setBlacklisted(false);
  
  // Verify all values
  EXPECT_EQ(frontier_->getUID(), 999);
  EXPECT_EQ(frontier_->getSize(), 50);
  EXPECT_DOUBLE_EQ(frontier_->getGoalPoint().x, 10.0);
  EXPECT_DOUBLE_EQ(frontier_->getGoalPoint().y, 20.0);
  EXPECT_DOUBLE_EQ(frontier_->getArrivalInformation(), 0.95);
  EXPECT_DOUBLE_EQ(frontier_->getPathLength(), 25.5);
  EXPECT_DOUBLE_EQ(frontier_->getPathLengthInM(), 25.5);
  EXPECT_DOUBLE_EQ(frontier_->getPathHeading(), M_PI / 6.0);
  EXPECT_DOUBLE_EQ(frontier_->getCost("distance"), 25.5);
  EXPECT_DOUBLE_EQ(frontier_->getCost("time"), 10.2);
  EXPECT_DOUBLE_EQ(frontier_->getWeightedCost(), 35.7);
  EXPECT_TRUE(frontier_->isAchievable());
  EXPECT_FALSE(frontier_->isBlacklisted());
  EXPECT_FALSE(frontier_->isFrontierNull());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
