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

#ifndef FRONTIER_HPP_
#define FRONTIER_HPP_

#include <vector>
#include <memory>
#include <map>
#include <limits>
#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav2_util/geometry_utils.hpp>

#include "roadmap_explorer/util/Logger.hpp"

template<typename T>
inline void setValue(std::shared_ptr<T> & ptr, T value)
{
  if (ptr == nullptr) {
    ptr = std::make_shared<T>(value);
  } else {
    *ptr = value;
  }
}

inline void setMapValue(
  std::shared_ptr<std::map<std::string, double>> & mapPtr, std::string key,
  double value)
{
  // Check if the map itself is initialized
  if (mapPtr == nullptr) {
    mapPtr = std::make_shared<std::map<std::string, double>>();
  }

  // Set or update the value for the given key in the map
  (*mapPtr)[key] = value;
}

/**
 * @class Frontier
 * @brief Represents an exploration frontier with associated costs and path information.
 *
 * A frontier is a boundary between explored and unexplored space in a map.
 * This class encapsulates all the information needed for frontier-based exploration,
 * including the frontier's location, achievability, information gain, path metrics,
 * and various cost components used for selecting the next exploration goal.
 *
 * The class uses shared pointers internally to allow efficient sharing and copying
 * of frontier objects while maintaining consistent state across references.
 */
class Frontier
{
private:
  // Variables not unique to a robot
  std::shared_ptr<size_t> unique_id;
  std::shared_ptr<int> size;
  std::shared_ptr<geometry_msgs::msg::Point> goal_point;
  std::shared_ptr<geometry_msgs::msg::Quaternion> best_orientation;
  std::shared_ptr<double> theta_s_star;
  std::shared_ptr<double> information;

  // Variables unique to a robot
  std::shared_ptr<double> path_length;
  std::shared_ptr<double> path_length_m;
  std::shared_ptr<double> path_heading;
  std::shared_ptr<bool> is_achievable;
  std::shared_ptr<bool> is_blacklisted;

  // Individual costs
  std::shared_ptr<std::map<std::string, double>> costs;
  // Weighted cost
  std::shared_ptr<double> weighted_cost;

public:
  // Constructor
  Frontier();

  void setUID(size_t uid);

  void setSize(int sz);

  void setGoalPoint(geometry_msgs::msg::Point gp);

  void setGoalPoint(double x, double y);

  void setGoalOrientation(double theta);

  void setArrivalInformation(double info);

  void setPathLength(double pl);

  void setPathLengthInM(double pl);

  void setPathHeading(double heading_rad);

  void setCost(std::string costName, double value);

  void setWeightedCost(double cost);

  void setAchievability(bool value);

  void setBlacklisted(bool value);

  bool operator==(const Frontier & other) const;

  size_t getUID() const;

  int getSize() const;

  const geometry_msgs::msg::Point & getGoalPoint() const;

  const geometry_msgs::msg::Quaternion & getGoalOrientation() const;

  double getArrivalInformation() const;

  double getPathLength() const;

  double getPathLengthInM() const;

  double getPathHeading() const;

  double getCost(const std::string & costName) const;

  double getWeightedCost() const;

  bool isAchievable() const;

  bool isBlacklisted() const;

  bool isFrontierNull() const;

  bool operator<(const Frontier & other) const
  {
    return getUID() < other.getUID();
  }

  friend std::ostream & operator<<(std::ostream & os, const Frontier & obj)
  {
    // Customize the output format here
    os << "Frontier(x: " << obj.getGoalPoint().x << ", y: " << obj.getGoalPoint().y << ")";
    // os << "Frontier Path Length(PL: " << obj.getPathLength() << ", PLm: " << obj.getPathLengthInM() << ")";
    return os;
  }
};

using FrontierPtr = std::shared_ptr<Frontier>;

// Custom equality function
struct FrontierGoalPointEquality
{
  bool operator()(const FrontierPtr & lhs, const FrontierPtr & rhs) const
  {
    return lhs->getGoalPoint() == rhs->getGoalPoint();
  }
};

inline size_t generateUID(const FrontierPtr & output)
{
  std::hash<double> hash_fn;
  // Hash each double value
  std::size_t hash1 = hash_fn(output->getGoalPoint().x);
  std::size_t hash2 = hash_fn(output->getGoalPoint().y);

  // return hash1 ^ (hash2 << 1);
  return hash1 ^ (hash2 << 1);
}

struct FrontierHash
{
  size_t operator()(const FrontierPtr & key) const
  {
    // Calculate hash based on some combination of member variables
    size_t hash = 0;
    hash = std::hash<double>()(key->getGoalPoint().x) ^
      (std::hash<double>()(key->getGoalPoint().y) << 1);
    return hash;
  }
};
#endif
