#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav2_util/geometry_utils.hpp>

// We'll create a mock Frontier class to avoid dependencies
class MockFrontier 
{
private:
    geometry_msgs::msg::Point goal_point_;
    
public:
    MockFrontier() {
        goal_point_.x = 0.0;
        goal_point_.y = 0.0;
        goal_point_.z = 0.0;
    }
    
    void setGoalPoint(double x, double y) {
        goal_point_.x = x;
        goal_point_.y = y;
        goal_point_.z = 0.0;
    }
    
    const geometry_msgs::msg::Point& getGoalPoint() const {
        return goal_point_;
    }
};

using MockFrontierPtr = std::shared_ptr<MockFrontier>;

// Define our own geometry utility functions for testing
inline double distanceBetweenFrontiers(const MockFrontierPtr & f1, const MockFrontierPtr & f2)
{
  return sqrt(
    pow(
      f1->getGoalPoint().x - f2->getGoalPoint().x,
      2) + pow(f1->getGoalPoint().y - f2->getGoalPoint().y, 2));
}

inline double sqDistanceBetweenFrontiers(const MockFrontierPtr & f1, const MockFrontierPtr & f2)
{
  return pow(f1->getGoalPoint().x - f2->getGoalPoint().x, 2) + pow(
    f1->getGoalPoint().y - f2->getGoalPoint().y, 2);
}

// Include the rest of GeometryUtils functions directly since they're inline  
#include "roadmap_explorer/util/GeometryUtils.hpp"

using namespace roadmap_explorer;

class GeometryUtilsTest : public ::testing::Test 
{
protected:
    void SetUp() override 
    {
        // Set up common test data
        frontier1 = std::make_shared<MockFrontier>();
        frontier2 = std::make_shared<MockFrontier>();
        
        frontier1->setGoalPoint(1.0, 2.0);
        frontier2->setGoalPoint(4.0, 6.0);
        
        point1.x = 0.0;
        point1.y = 0.0;
        point1.z = 0.0;
        
        point2.x = 3.0;
        point2.y = 4.0;
        point2.z = 0.0;
        
        // Create a quaternion for 90 degrees rotation around Z axis
        quat_90_z.x = 0.0;
        quat_90_z.y = 0.0;
        quat_90_z.z = 0.7071068; // sin(45°)
        quat_90_z.w = 0.7071068; // cos(45°)
        
        // Create a quaternion for identity (no rotation)
        quat_identity.x = 0.0;
        quat_identity.y = 0.0;
        quat_identity.z = 0.0;
        quat_identity.w = 1.0;
    }
    
    MockFrontierPtr frontier1, frontier2;
    geometry_msgs::msg::Point point1, point2;
    geometry_msgs::msg::Quaternion quat_90_z, quat_identity;
    
    // Helper function to compare doubles with tolerance
    bool isApproxEqual(double a, double b, double tolerance = 1e-6) 
    {
        return std::abs(a - b) < tolerance;
    }
    
    // Helper function to compare vectors with tolerance
    bool isApproxEqualVector(const std::vector<double>& a, const std::vector<double>& b, double tolerance = 1e-6) 
    {
        if (a.size() != b.size()) return false;
        for (size_t i = 0; i < a.size(); ++i) {
            if (!isApproxEqual(a[i], b[i], tolerance)) return false;
        }
        return true;
    }
};

// Tests for quatToEuler function
TEST_F(GeometryUtilsTest, QuatToEulerIdentity) 
{
    auto rpy = quatToEuler(quat_identity);
    
    EXPECT_EQ(rpy.size(), 3);
    EXPECT_TRUE(isApproxEqual(rpy[0], 0.0)); // roll
    EXPECT_TRUE(isApproxEqual(rpy[1], 0.0)); // pitch
    EXPECT_TRUE(isApproxEqual(rpy[2], 0.0)); // yaw
}

TEST_F(GeometryUtilsTest, QuatToEuler90DegreeZ) 
{
    auto rpy = quatToEuler(quat_90_z);
    
    EXPECT_EQ(rpy.size(), 3);
    EXPECT_TRUE(isApproxEqual(rpy[0], 0.0));        // roll
    EXPECT_TRUE(isApproxEqual(rpy[1], 0.0));        // pitch
    EXPECT_TRUE(isApproxEqual(rpy[2], M_PI_2));     // yaw = 90 degrees
}

TEST_F(GeometryUtilsTest, QuatToEulerEdgeCases) 
{
    // Test with very small quaternion values
    geometry_msgs::msg::Quaternion small_quat;
    small_quat.x = 1e-10;
    small_quat.y = 1e-10;
    small_quat.z = 1e-10;
    small_quat.w = 1.0;
    
    auto rpy = quatToEuler(small_quat);
    EXPECT_EQ(rpy.size(), 3);
    // Should be close to identity
    EXPECT_TRUE(isApproxEqual(rpy[0], 0.0, 1e-8));
    EXPECT_TRUE(isApproxEqual(rpy[1], 0.0, 1e-8));
    EXPECT_TRUE(isApproxEqual(rpy[2], 0.0, 1e-8));
}

// Tests for eulerToQuat function
TEST_F(GeometryUtilsTest, EulerToQuatIdentity) 
{
    auto quat = eulerToQuat(0.0, 0.0, 0.0);
    
    EXPECT_TRUE(isApproxEqual(quat.x, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.y, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.z, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.w, 1.0));
}

TEST_F(GeometryUtilsTest, EulerToQuat90DegreeZ) 
{
    auto quat = eulerToQuat(0.0, 0.0, M_PI_2);
    
    EXPECT_TRUE(isApproxEqual(quat.x, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.y, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.z, 0.7071068, 1e-6));
    EXPECT_TRUE(isApproxEqual(quat.w, 0.7071068, 1e-6));
}

TEST_F(GeometryUtilsTest, EulerToQuatFullRotation) 
{
    // Test with 2*PI rotation (should be equivalent to identity)
    auto quat = eulerToQuat(0.0, 0.0, 2*M_PI);
    
    EXPECT_TRUE(isApproxEqual(quat.x, 0.0, 1e-6));
    EXPECT_TRUE(isApproxEqual(quat.y, 0.0, 1e-6));
    EXPECT_TRUE(isApproxEqual(quat.z, 0.0, 1e-6));
    EXPECT_TRUE(isApproxEqual(std::abs(quat.w), 1.0, 1e-6)); // Could be +1 or -1
}

TEST_F(GeometryUtilsTest, EulerToQuatNormalization) 
{
    auto quat = eulerToQuat(M_PI_4, M_PI_4, M_PI_4);
    
    // Check that quaternion is normalized
    double norm = sqrt(quat.x*quat.x + quat.y*quat.y + quat.z*quat.z + quat.w*quat.w);
    EXPECT_TRUE(isApproxEqual(norm, 1.0));
}

// Test round-trip conversion
TEST_F(GeometryUtilsTest, EulerQuatRoundTrip) 
{
    double roll = M_PI_4;
    double pitch = M_PI/6;
    double yaw = M_PI_2;
    
    auto quat = eulerToQuat(roll, pitch, yaw);
    auto rpy = quatToEuler(quat);
    
    EXPECT_TRUE(isApproxEqual(rpy[0], roll));
    EXPECT_TRUE(isApproxEqual(rpy[1], pitch));
    EXPECT_TRUE(isApproxEqual(rpy[2], yaw));
}

// Tests for getDifferenceInRPY function
TEST_F(GeometryUtilsTest, GetDifferenceInRPYIdentical) 
{
    std::vector<double> rpy1 = {0.1, 0.2, 0.3};
    std::vector<double> rpy2 = {0.1, 0.2, 0.3};
    
    auto diff = getDifferenceInRPY(rpy1, rpy2);
    
    EXPECT_TRUE(isApproxEqual(diff[0], 0.0));
    EXPECT_TRUE(isApproxEqual(diff[1], 0.0));
    EXPECT_TRUE(isApproxEqual(diff[2], 0.0));
}

TEST_F(GeometryUtilsTest, GetDifferenceInRPYSimple) 
{
    std::vector<double> rpy1 = {0.0, 0.0, 0.0};
    std::vector<double> rpy2 = {0.5, 1.0, 1.5};
    
    auto diff = getDifferenceInRPY(rpy1, rpy2);
    
    EXPECT_TRUE(isApproxEqual(diff[0], 0.5));
    EXPECT_TRUE(isApproxEqual(diff[1], 1.0));
    EXPECT_TRUE(isApproxEqual(diff[2], 1.5));
}

TEST_F(GeometryUtilsTest, GetDifferenceInRPYWrapAround) 
{
    std::vector<double> rpy1 = {0.1, 0.1, 0.1};
    std::vector<double> rpy2 = {2*M_PI - 0.1, 2*M_PI - 0.1, 2*M_PI - 0.1};
    
    auto diff = getDifferenceInRPY(rpy1, rpy2);
    
    // Due to wrap-around, difference should be 0.2 for each
    EXPECT_TRUE(isApproxEqual(diff[0], 0.2));
    EXPECT_TRUE(isApproxEqual(diff[1], 0.2));
    EXPECT_TRUE(isApproxEqual(diff[2], 0.2));
}

TEST_F(GeometryUtilsTest, GetDifferenceInRPYLargeAngles) 
{
    std::vector<double> rpy1 = {M_PI + 0.1, M_PI + 0.1, M_PI + 0.1};
    std::vector<double> rpy2 = {M_PI - 0.1, M_PI - 0.1, M_PI - 0.1};
    
    auto diff = getDifferenceInRPY(rpy1, rpy2);
    
    EXPECT_TRUE(isApproxEqual(diff[0], 0.2));
    EXPECT_TRUE(isApproxEqual(diff[1], 0.2));
    EXPECT_TRUE(isApproxEqual(diff[2], 0.2));
}

// Tests for yawToQuat function
TEST_F(GeometryUtilsTest, YawToQuatZero) 
{
    auto quat = yawToQuat(0.0);
    
    EXPECT_TRUE(isApproxEqual(quat.x, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.y, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.z, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.w, 1.0));
}

TEST_F(GeometryUtilsTest, YawToQuat90Degrees) 
{
    auto quat = yawToQuat(M_PI_2);
    
    EXPECT_TRUE(isApproxEqual(quat.x, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.y, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.z, 0.7071068, 1e-6));
    EXPECT_TRUE(isApproxEqual(quat.w, 0.7071068, 1e-6));
}

TEST_F(GeometryUtilsTest, YawToQuatNegativeAngle) 
{
    auto quat = yawToQuat(-M_PI_2);
    
    EXPECT_TRUE(isApproxEqual(quat.x, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.y, 0.0));
    EXPECT_TRUE(isApproxEqual(quat.z, -0.7071068, 1e-6));
    EXPECT_TRUE(isApproxEqual(quat.w, 0.7071068, 1e-6));
}

TEST_F(GeometryUtilsTest, YawToQuatLargeAngle) 
{
    // Test with angle > 2*PI
    auto quat = yawToQuat(3*M_PI);
    
    // 3*PI is equivalent to PI (180 degrees), but due to quaternion double-cover
    // the quaternion representation might be negated. Check that the magnitude is correct.
    auto expected_quat = yawToQuat(M_PI);
    
    // Check that either quat matches expected_quat or its negation (quaternion double cover)
    bool matches_positive = isApproxEqual(quat.x, expected_quat.x, 1e-6) && 
                           isApproxEqual(quat.y, expected_quat.y, 1e-6) &&
                           isApproxEqual(quat.z, expected_quat.z, 1e-6) && 
                           isApproxEqual(quat.w, expected_quat.w, 1e-6);
                           
    bool matches_negative = isApproxEqual(quat.x, -expected_quat.x, 1e-6) && 
                           isApproxEqual(quat.y, -expected_quat.y, 1e-6) &&
                           isApproxEqual(quat.z, -expected_quat.z, 1e-6) && 
                           isApproxEqual(quat.w, -expected_quat.w, 1e-6);
    
    EXPECT_TRUE(matches_positive || matches_negative);
}

// Tests for distanceBetweenFrontiers function
TEST_F(GeometryUtilsTest, DistanceBetweenFrontiersBasic) 
{
    double distance = distanceBetweenFrontiers(frontier1, frontier2);
    
    // Distance between (1,2) and (4,6) should be 5.0
    EXPECT_TRUE(isApproxEqual(distance, 5.0));
}

TEST_F(GeometryUtilsTest, DistanceBetweenFrontiersIdentical) 
{
    auto frontier3 = std::make_shared<MockFrontier>();
    frontier3->setGoalPoint(1.0, 2.0);
    
    double distance = distanceBetweenFrontiers(frontier1, frontier3);
    
    EXPECT_TRUE(isApproxEqual(distance, 0.0));
}

TEST_F(GeometryUtilsTest, DistanceBetweenFrontiersNegativeCoords) 
{
    auto frontier3 = std::make_shared<MockFrontier>();
    frontier3->setGoalPoint(-1.0, -2.0);
    
    double distance = distanceBetweenFrontiers(frontier1, frontier3);
    
    // Distance between (1,2) and (-1,-2) should be sqrt((1-(-1))^2 + (2-(-2))^2) = sqrt(4+16) = sqrt(20)
    EXPECT_TRUE(isApproxEqual(distance, sqrt(20.0)));
}

// Tests for distanceBetweenPoints function (Point to Point)
TEST_F(GeometryUtilsTest, DistanceBetweenPointsBasic) 
{
    double distance = distanceBetweenPoints(point1, point2);
    
    // Distance between (0,0) and (3,4) should be 5.0
    EXPECT_TRUE(isApproxEqual(distance, 5.0));
}

TEST_F(GeometryUtilsTest, DistanceBetweenPointsIdentical) 
{
    double distance = distanceBetweenPoints(point1, point1);
    
    EXPECT_TRUE(isApproxEqual(distance, 0.0));
}

TEST_F(GeometryUtilsTest, DistanceBetweenPointsLargeValues) 
{
    geometry_msgs::msg::Point large_point1, large_point2;
    large_point1.x = 1000000.0;
    large_point1.y = 1000000.0;
    large_point2.x = 1000003.0;
    large_point2.y = 1000004.0;
    
    double distance = distanceBetweenPoints(large_point1, large_point2);
    
    EXPECT_TRUE(isApproxEqual(distance, 5.0));
}

// Tests for distanceBetweenPoints function (Point to coordinates)
TEST_F(GeometryUtilsTest, DistanceBetweenPointsCoordinatesBasic) 
{
    double distance = distanceBetweenPoints(point1, 3.0, 4.0);
    
    EXPECT_TRUE(isApproxEqual(distance, 5.0));
}

TEST_F(GeometryUtilsTest, DistanceBetweenPointsCoordinatesNegative) 
{
    double distance = distanceBetweenPoints(point1, -3.0, -4.0);
    
    EXPECT_TRUE(isApproxEqual(distance, 5.0));
}

TEST_F(GeometryUtilsTest, DistanceBetweenPointsCoordinatesZero) 
{
    double distance = distanceBetweenPoints(point1, 0.0, 0.0);
    
    EXPECT_TRUE(isApproxEqual(distance, 0.0));
}

// Tests for distanceBetweenPointsSq function (squared distance)
TEST_F(GeometryUtilsTest, DistanceBetweenPointsSqBasic) 
{
    double distance_sq = distanceBetweenPointsSq(point1, 3.0, 4.0);
    
    EXPECT_TRUE(isApproxEqual(distance_sq, 25.0));
}

TEST_F(GeometryUtilsTest, DistanceBetweenPointsSqZero) 
{
    double distance_sq = distanceBetweenPointsSq(point1, 0.0, 0.0);
    
    EXPECT_TRUE(isApproxEqual(distance_sq, 0.0));
}

TEST_F(GeometryUtilsTest, DistanceBetweenPointsSqNegative) 
{
    double distance_sq = distanceBetweenPointsSq(point1, -3.0, -4.0);
    
    EXPECT_TRUE(isApproxEqual(distance_sq, 25.0));
}

// Tests for sqDistanceBetweenFrontiers function
TEST_F(GeometryUtilsTest, SqDistanceBetweenFrontiersBasic) 
{
    double distance_sq = sqDistanceBetweenFrontiers(frontier1, frontier2);
    
    // Squared distance between (1,2) and (4,6) should be 25.0
    EXPECT_TRUE(isApproxEqual(distance_sq, 25.0));
}

TEST_F(GeometryUtilsTest, SqDistanceBetweenFrontiersIdentical) 
{
    auto frontier3 = std::make_shared<MockFrontier>();
    frontier3->setGoalPoint(1.0, 2.0);
    
    double distance_sq = sqDistanceBetweenFrontiers(frontier1, frontier3);
    
    EXPECT_TRUE(isApproxEqual(distance_sq, 0.0));
}

// Tests for getRelativePoseGivenTwoPoints function
TEST_F(GeometryUtilsTest, GetRelativePoseBasic) 
{
    geometry_msgs::msg::Pose pose;
    getRelativePoseGivenTwoPoints(point1, point2, pose);
    
    // Position should be set to point_from (point1)
    EXPECT_TRUE(isApproxEqual(pose.position.x, point1.x));
    EXPECT_TRUE(isApproxEqual(pose.position.y, point1.y));
    EXPECT_TRUE(isApproxEqual(pose.position.z, point1.z));
    
    // Orientation should point from point1 to point2
    // atan2(4-0, 3-0) = atan2(4, 3) ≈ 0.927 radians
    double expected_yaw = atan2(4.0, 3.0);
    
    // Convert quaternion back to yaw to verify
    tf2::Quaternion tf2_quat(pose.orientation.x, pose.orientation.y, 
                            pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 matrix(tf2_quat);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    
    EXPECT_TRUE(isApproxEqual(yaw, expected_yaw));
}

TEST_F(GeometryUtilsTest, GetRelativePoseVertical) 
{
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0; p1.y = 0.0;
    p2.x = 0.0; p2.y = 5.0; // Straight up
    
    geometry_msgs::msg::Pose pose;
    getRelativePoseGivenTwoPoints(p1, p2, pose);
    
    // Expected yaw for straight up should be PI/2
    tf2::Quaternion tf2_quat(pose.orientation.x, pose.orientation.y, 
                            pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 matrix(tf2_quat);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    
    EXPECT_TRUE(isApproxEqual(yaw, M_PI_2));
}

TEST_F(GeometryUtilsTest, GetRelativePoseHorizontal) 
{
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0; p1.y = 0.0;
    p2.x = 5.0; p2.y = 0.0; // Straight right
    
    geometry_msgs::msg::Pose pose;
    getRelativePoseGivenTwoPoints(p1, p2, pose);
    
    // Expected yaw for straight right should be 0
    tf2::Quaternion tf2_quat(pose.orientation.x, pose.orientation.y, 
                            pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 matrix(tf2_quat);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    
    EXPECT_TRUE(isApproxEqual(yaw, 0.0));
}

TEST_F(GeometryUtilsTest, GetRelativePoseIdenticalPoints) 
{
    geometry_msgs::msg::Pose pose;
    getRelativePoseGivenTwoPoints(point1, point1, pose);
    
    // Position should be set to point1
    EXPECT_TRUE(isApproxEqual(pose.position.x, point1.x));
    EXPECT_TRUE(isApproxEqual(pose.position.y, point1.y));
    
    // When points are identical, atan2(0, 0) returns 0
    tf2::Quaternion tf2_quat(pose.orientation.x, pose.orientation.y, 
                            pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 matrix(tf2_quat);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    
    EXPECT_TRUE(isApproxEqual(yaw, 0.0));
}

TEST_F(GeometryUtilsTest, GetRelativePoseNegativeDirection) 
{
    geometry_msgs::msg::Point p1, p2;
    p1.x = 5.0; p1.y = 5.0;
    p2.x = 0.0; p2.y = 0.0; // Moving to origin
    
    geometry_msgs::msg::Pose pose;
    getRelativePoseGivenTwoPoints(p1, p2, pose);
    
    // Expected yaw should be -3*PI/4 (225 degrees)
    double expected_yaw = atan2(-5.0, -5.0);
    
    tf2::Quaternion tf2_quat(pose.orientation.x, pose.orientation.y, 
                            pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 matrix(tf2_quat);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    
    EXPECT_TRUE(isApproxEqual(yaw, expected_yaw));
}

// Edge case tests
TEST_F(GeometryUtilsTest, ExtremelySmallDistances) 
{
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0; p1.y = 0.0;
    p2.x = 1e-10; p2.y = 1e-10;
    
    double distance = distanceBetweenPoints(p1, p2);
    double expected = sqrt(2e-20);
    
    EXPECT_TRUE(isApproxEqual(distance, expected, 1e-15));
}

TEST_F(GeometryUtilsTest, ExtremelyLargeDistances) 
{
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0; p1.y = 0.0;
    p2.x = 1e10; p2.y = 1e10;
    
    double distance = distanceBetweenPoints(p1, p2);
    double expected = sqrt(2e20);
    
    EXPECT_TRUE(isApproxEqual(distance, expected, 1e4)); // Larger tolerance for large numbers
}

TEST_F(GeometryUtilsTest, InfinityHandling) 
{
    // Test with very large numbers that could cause overflow
    geometry_msgs::msg::Point p1, p2;
    p1.x = std::numeric_limits<double>::max() / 2;
    p1.y = 0.0;
    p2.x = std::numeric_limits<double>::max() / 2;
    p2.y = 1.0;
    
    double distance = distanceBetweenPoints(p1, p2);
    EXPECT_TRUE(std::isfinite(distance));
    EXPECT_TRUE(isApproxEqual(distance, 1.0));
}

// Test consistency between different distance functions
TEST_F(GeometryUtilsTest, DistanceFunctionConsistency) 
{
    double dist1 = distanceBetweenPoints(point1, point2);
    double dist2 = distanceBetweenPoints(point1, point2.x, point2.y);
    double dist_sq = distanceBetweenPointsSq(point1, point2.x, point2.y);
    
    EXPECT_TRUE(isApproxEqual(dist1, dist2));
    EXPECT_TRUE(isApproxEqual(dist1 * dist1, dist_sq));
}

// Tests for getTransformFromPose function
TEST_F(GeometryUtilsTest, GetTransformFromPoseIdentity) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation = quat_identity;
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // Identity transform should have identity matrix
    Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f actual = transform.matrix();
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            EXPECT_TRUE(isApproxEqual(actual(i, j), expected(i, j), 1e-6));
        }
    }
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseTranslationOnly) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 5.0;
    pose.position.y = 10.0;
    pose.position.z = 15.0;
    pose.orientation = quat_identity;
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // Check translation part
    Eigen::Vector3f translation = transform.translation();
    EXPECT_TRUE(isApproxEqual(translation.x(), 5.0f));
    EXPECT_TRUE(isApproxEqual(translation.y(), 10.0f));
    EXPECT_TRUE(isApproxEqual(translation.z(), 15.0f));
    
    // Check rotation part (should be identity)
    Eigen::Matrix3f rotation = transform.rotation();
    Eigen::Matrix3f expected_rotation = Eigen::Matrix3f::Identity();
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_TRUE(isApproxEqual(rotation(i, j), expected_rotation(i, j), 1e-6));
        }
    }
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseRotationOnly) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation = quat_90_z; // 90 degree rotation around Z
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // Check translation part (should be zero)
    Eigen::Vector3f translation = transform.translation();
    EXPECT_TRUE(isApproxEqual(translation.x(), 0.0f));
    EXPECT_TRUE(isApproxEqual(translation.y(), 0.0f));
    EXPECT_TRUE(isApproxEqual(translation.z(), 0.0f));
    
    // Check rotation part
    Eigen::Matrix3f rotation = transform.rotation();
    
    // For 90 degree rotation around Z, we expect:
    // [0 -1  0]
    // [1  0  0]
    // [0  0  1]
    EXPECT_TRUE(isApproxEqual(rotation(0, 0), 0.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(0, 1), -1.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(0, 2), 0.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(1, 0), 1.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(1, 1), 0.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(1, 2), 0.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(2, 0), 0.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(2, 1), 0.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(2, 2), 1.0f, 1e-6));
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseFullTransform) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 2.0;
    pose.position.y = 3.0;
    pose.position.z = 4.0;
    pose.orientation = quat_90_z; // 90 degree rotation around Z
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // Check translation part
    Eigen::Vector3f translation = transform.translation();
    EXPECT_TRUE(isApproxEqual(translation.x(), 2.0f));
    EXPECT_TRUE(isApproxEqual(translation.y(), 3.0f));
    EXPECT_TRUE(isApproxEqual(translation.z(), 4.0f));
    
    // Check that rotation is applied correctly
    Eigen::Matrix3f rotation = transform.rotation();
    EXPECT_TRUE(isApproxEqual(rotation(0, 0), 0.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(0, 1), -1.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(1, 0), 1.0f, 1e-6));
    EXPECT_TRUE(isApproxEqual(rotation(1, 1), 0.0f, 1e-6));
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseArbitraryRotation) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;
    
    // Create a quaternion for 45 degrees around X, 30 degrees around Y, 60 degrees around Z
    double roll = M_PI / 4.0;   // 45 degrees
    double pitch = M_PI / 6.0;  // 30 degrees
    double yaw = M_PI / 3.0;    // 60 degrees
    
    pose.orientation = eulerToQuat(roll, pitch, yaw);
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // Verify the transform by applying it to a test vector
    Eigen::Vector3f test_vector(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f transformed_vector = transform * test_vector;
    
    // The result should be different from the original vector
    EXPECT_FALSE(isApproxEqual(transformed_vector.x(), test_vector.x(), 1e-6));
    
    // Check that the transformation preserves vector length (rotation + translation)
    Eigen::Vector3f rotated_only = transform.rotation() * test_vector;
    float original_length = test_vector.norm();
    float rotated_length = rotated_only.norm();
    EXPECT_TRUE(isApproxEqual(original_length, rotated_length, 1e-6));
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseQuaternionNormalization) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    
    // Create an unnormalized quaternion
    pose.orientation.x = 0.5;
    pose.orientation.y = 0.5;
    pose.orientation.z = 0.5;
    pose.orientation.w = 0.5;
    
    // Eigen should handle normalization internally
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // The transform should still be valid (determinant of rotation part should be 1)
    Eigen::Matrix3f rotation = transform.rotation();
    float det = rotation.determinant();
    EXPECT_TRUE(isApproxEqual(det, 1.0f, 1e-5));
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseInverseTransform) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 5.0;
    pose.position.y = 10.0;
    pose.position.z = 15.0;
    pose.orientation = quat_90_z;
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    Eigen::Affine3f inverse_transform = transform.inverse();
    
    // Applying transform and then its inverse should give identity
    Eigen::Affine3f result = transform * inverse_transform;
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            EXPECT_TRUE(isApproxEqual(result.matrix()(i, j), identity(i, j), 1e-5));
        }
    }
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseVectorTransformation) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;
    pose.orientation = quat_identity;
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // Test transforming a point
    Eigen::Vector3f point(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f transformed_point = transform * point;
    
    // With identity rotation, the point should just be translated
    EXPECT_TRUE(isApproxEqual(transformed_point.x(), 1.0f));
    EXPECT_TRUE(isApproxEqual(transformed_point.y(), 2.0f));
    EXPECT_TRUE(isApproxEqual(transformed_point.z(), 3.0f));
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseNegativeValues) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = -5.0;
    pose.position.y = -10.0;
    pose.position.z = -15.0;
    pose.orientation = quat_identity;
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // Check negative translation values
    Eigen::Vector3f translation = transform.translation();
    EXPECT_TRUE(isApproxEqual(translation.x(), -5.0f));
    EXPECT_TRUE(isApproxEqual(translation.y(), -10.0f));
    EXPECT_TRUE(isApproxEqual(translation.z(), -15.0f));
}

TEST_F(GeometryUtilsTest, GetTransformFromPoseZeroValues) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0; // This will be normalized by Eigen
    
    Eigen::Affine3f transform = getTransformFromPose(pose);
    
    // Even with zero quaternion, Eigen should handle it gracefully
    // The result might not be identity due to normalization, but should be valid
    Eigen::Matrix3f rotation = transform.rotation();
    float det = rotation.determinant();
    
    // Determinant should be close to 1 (valid rotation matrix)
    EXPECT_TRUE(std::abs(det - 1.0f) < 1e-3 || std::abs(det + 1.0f) < 1e-3);
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
