#ifndef GEOMETRYUTILS_HPP
#define GEOMETRYUTILS_HPP

#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include "roadmap_explorer/Frontier.hpp"

namespace roadmap_explorer
{

struct Point2D
{
  double x;
  double y;
};

inline std::vector<double> quatToEuler(geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf2_quaternion(
    quat.x, quat.y, quat.z, quat.w);

  // Convert tf2 quaternion to Euler angles
  tf2::Matrix3x3 matrix(tf2_quaternion);
  std::vector<double> rpy = {0, 0, 0};
  matrix.getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}

inline std::vector<double> quatToEuler0To2MPI(geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf2_quaternion(
    quat.x, quat.y, quat.z, quat.w);

  // Convert tf2 quaternion to Euler angles
  tf2::Matrix3x3 matrix(tf2_quaternion);
  std::vector<double> rpy = {0, 0, 0};
  matrix.getRPY(rpy[0], rpy[1], rpy[2]);
  if (rpy[0] < 0) {rpy[0] = rpy[0] + 2 * M_PI;}
  if (rpy[1] < 0) {rpy[1] = rpy[1] + 2 * M_PI;}
  if (rpy[2] < 0) {rpy[2] = rpy[2] + 2 * M_PI;}
  return rpy;
}

inline geometry_msgs::msg::Quaternion eulerToQuat(double roll, double pitch, double yaw)
{
  // Create a tf2 quaternion and set it based on Euler angles
  tf2::Quaternion tf2_quaternion;
  tf2_quaternion.setRPY(roll, pitch, yaw);
  tf2_quaternion.normalize();       // Ensure the quaternion is normalized

  // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg.x = tf2_quaternion.x();
  quat_msg.y = tf2_quaternion.y();
  quat_msg.z = tf2_quaternion.z();
  quat_msg.w = tf2_quaternion.w();

  return quat_msg;
}

inline std::vector<double> getDifferenceInRPY(std::vector<double> rpy1, std::vector<double> rpy2)
{
  std::vector<double> rpy = {0, 0, 0};
  rpy[0] = abs(rpy1[0] - rpy2[0]);
  rpy[1] = abs(rpy1[1] - rpy2[1]);
  rpy[2] = abs(rpy1[2] - rpy2[2]);
  if (rpy[0] > M_PI) {rpy[0] = 2 * M_PI - rpy[0];}
  if (rpy[1] > M_PI) {rpy[1] = 2 * M_PI - rpy[1];}
  if (rpy[2] > M_PI) {rpy[2] = 2 * M_PI - rpy[2];}
  return rpy;
}

inline geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);       // Set roll, pitch, and yaw. Here, roll and pitch are 0.

  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg.x = quaternion.x();
  quat_msg.y = quaternion.y();
  quat_msg.z = quaternion.z();
  quat_msg.w = quaternion.w();

  return quat_msg;
}

inline double distanceBetweenFrontiers(const FrontierPtr & f1, const FrontierPtr & f2)
{
  return sqrt(
    pow(
      f1->getGoalPoint().x - f2->getGoalPoint().x,
      2) + pow(f1->getGoalPoint().y - f2->getGoalPoint().y, 2));
}

inline double distanceBetweenPoints(
  const geometry_msgs::msg::Point & f1,
  const geometry_msgs::msg::Point & f2)
{
  return sqrt(pow(f1.x - f2.x, 2) + pow(f1.y - f2.y, 2));
}

inline double distanceBetweenPoints(
  const geometry_msgs::msg::Point & f1, const double x2,
  const double y2)
{
  return sqrt(pow(f1.x - x2, 2) + pow(f1.y - y2, 2));
}

inline double distanceBetweenPointsSq(
  const geometry_msgs::msg::Point & f1, const double x2,
  const double y2)
{
  return ((f1.x - x2) * (f1.x - x2)) + ((f1.y - y2) * (f1.y - y2));
}

inline double sqDistanceBetweenFrontiers(const FrontierPtr & f1, const FrontierPtr & f2)
{
  return pow(f1->getGoalPoint().x - f2->getGoalPoint().x, 2) + pow(
    f1->getGoalPoint().y - f2->getGoalPoint().y, 2);
}

inline void getRelativePoseGivenTwoPoints(
  const geometry_msgs::msg::Point & point_from,
  const geometry_msgs::msg::Point & point_to,
  geometry_msgs::msg::Pose & oriented_pose)
{
  // size_t plan_size = plan.poses.size();
  // if (plan_size == 1) {
  //   plan.poses.back().pose.orientation = start.orientation;
  // } else if (plan_size > 1) {
  double dx, dy, theta;
  dx = point_to.x - point_from.x;
  dy = point_to.y - point_from.y;
  theta = atan2(dy, dx);
  oriented_pose.position = point_from;
  oriented_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
}

}
#endif
