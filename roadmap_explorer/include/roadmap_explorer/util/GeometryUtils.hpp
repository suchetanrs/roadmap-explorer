/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#ifndef GEOMETRYUTILS_HPP_
#define GEOMETRYUTILS_HPP_

#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav2_util/geometry_utils.hpp>

#include "roadmap_explorer/Frontier.hpp"

namespace roadmap_explorer
{

inline std::vector<double> quatToEuler(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf2_quaternion(
    quat.x, quat.y, quat.z, quat.w);

  // Convert tf2 quaternion to Euler angles
  tf2::Matrix3x3 matrix(tf2_quaternion);
  std::vector<double> rpy = {0, 0, 0};
  matrix.getRPY(rpy[0], rpy[1], rpy[2]);
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

inline std::vector<double> getDifferenceInRPY(
  const std::vector<double> & rpy1,
  const std::vector<double> & rpy2)
{
  std::vector<double> rpy = {0, 0, 0};
  rpy[0] = std::fabs(rpy1[0] - rpy2[0]);
  rpy[1] = std::fabs(rpy1[1] - rpy2[1]);
  rpy[2] = std::fabs(rpy1[2] - rpy2[2]);
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

inline Eigen::Affine3f getTransformFromPose(const geometry_msgs::msg::Pose & pose)
{
  // Extract translation and rotation from the pose message
  Eigen::Vector3f translation(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaternionf rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);

  // Construct the transformation matrix
  Eigen::Affine3f transform = Eigen::Translation3f(translation) * Eigen::Quaternionf(rotation);

  return transform;
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
  double dx, dy, theta;
  dx = point_to.x - point_from.x;
  dy = point_to.y - point_from.y;
  theta = atan2(dy, dx);
  oriented_pose.position = point_from;
  oriented_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
}

}
#endif // GEOMETRYUTILS_HPP_
