// Include necessary headers
#include <roadmap_explorer/Frontier.hpp>

// Constructor
Frontier::Frontier()
{
  unique_id = nullptr;
  size = nullptr;
  goal_point = nullptr;
  best_orientation = nullptr;
  theta_s_star = nullptr;
  information = nullptr;
  path_length = nullptr;
  path_length_m = nullptr;
  path_heading = nullptr;
  is_achievable = std::make_shared<bool>(true);
  is_blacklisted = std::make_shared<bool>(false);
  costs = nullptr;
  weighted_cost = nullptr;
}

void Frontier::setUID(size_t uid)
{
  setValue(unique_id, uid);
  // unique_id = std::make_shared<size_t>(uid);
}

void Frontier::setSize(int sz)
{
  setValue(size, sz);
  // size = std::make_shared<int>(sz);
}

void Frontier::setGoalPoint(geometry_msgs::msg::Point gp)
{
  setValue(goal_point, gp);
  // goal_point = std::make_shared<geometry_msgs::msg::Point>(gp);
}

void Frontier::setGoalPoint(double x, double y)
{
  geometry_msgs::msg::Point pnt;
  pnt.x = x;
  pnt.y = y;
  setValue(goal_point, pnt);
  // goal_point = std::make_shared<geometry_msgs::msg::Point>(pnt);
}

void Frontier::setGoalOrientation(double theta)
{
  setValue(theta_s_star, theta);
  // theta_s_star = std::make_shared<double>(theta);
  setValue(best_orientation, nav2_util::geometry_utils::orientationAroundZAxis(theta));
  // best_orientation = std::make_shared<geometry_msgs::msg::Quaternion>(nav2_util::geometry_utils::orientationAroundZAxis(theta));
}

void Frontier::setArrivalInformation(double info)
{
  setValue(information, info);
  // information = std::make_shared<double>(info);
}

void Frontier::setPathLength(double pl)
{
  setValue(path_length, pl);
  // path_length = std::make_shared<double>(pl);
}

void Frontier::setPathLengthInM(double pl)
{
  setValue(path_length_m, pl);
  // path_length_m = std::make_shared<double>(pl);
}

void Frontier::setPathHeading(double heading_rad)
{
  setValue(path_heading, heading_rad);
  // path_heading = std::make_shared<double>(heading_rad);
}

void Frontier::setCost(std::string costName, double value)
{
  setMapValue(costs, costName, value);
  // costs[costName] = value;
}

void Frontier::setWeightedCost(double cost)
{
  setValue(weighted_cost, cost);
  // weighted_cost = std::make_shared<double>(cost);
}

void Frontier::setAchievability(bool value)
{
  setValue(is_achievable, value);
  // is_achievable = value;
}

void Frontier::setBlacklisted(bool value)
{
  setValue(is_blacklisted, value);
}

// Equality operator definition
bool Frontier::operator==(const Frontier & other) const
{
  // if (!unique_id)
  //     throw RoadmapExplorerException("Cannot check equality. unique_id is null");
  // if (!size)
  //     throw RoadmapExplorerException("Cannot check equality. size is null");
  if (!goal_point || !other.goal_point) {
    throw RoadmapExplorerException("Cannot check equality. goal_point is null");
  }
  // if (!best_orientation)
  //     throw RoadmapExplorerException("Cannot check equality. best_orientation is null");

  // if (*unique_id != *other.unique_id ||
  //     *size != *other.size)
  // {
  //     return false;
  // }

  if (*goal_point != *other.goal_point) {
    return false;
  }

  // if (*best_orientation != *other.best_orientation)
  // {
  //     return false;
  // }

  return true;
}

size_t Frontier::getUID() const
{
  if (unique_id == nullptr) {
    LOG_CRITICAL("is null for: " << getGoalPoint().x << ", " << getGoalPoint().y);
    throw RoadmapExplorerException("Goal uid frontier property is null");
  }
  return *unique_id;
}

int Frontier::getSize() const
{
  if (size == nullptr) {
    throw RoadmapExplorerException("Size frontier property is null");
  }
  return *size;
}

const geometry_msgs::msg::Point & Frontier::getGoalPoint() const
{
  if (goal_point == nullptr) {
    throw RoadmapExplorerException("Goal point frontier property is null");
  }
  return *goal_point;
}

const geometry_msgs::msg::Quaternion & Frontier::getGoalOrientation() const
{
  if (best_orientation == nullptr || theta_s_star == nullptr) {
    throw RoadmapExplorerException("Goal orientation frontier property is null");
  }
  return *best_orientation;
}

double Frontier::getArrivalInformation() const
{
  if (information == nullptr) {
    throw RoadmapExplorerException("Arrival information frontier property is null");
  }
  return *information;
}

double Frontier::getPathLength() const
{
  if (path_length == nullptr) {
    throw RoadmapExplorerException("Path length frontier property is null");
  }
  return *path_length;
}

double Frontier::getPathLengthInM() const
{
  if (path_length_m == nullptr) {
    throw RoadmapExplorerException("Path length in m frontier property is null");
  }
  return *path_length_m;
}

double Frontier::getPathHeading() const
{
  if (path_heading == nullptr) {
    throw RoadmapExplorerException("Path heading frontier property is null");
  }
  return *path_heading;
}

double Frontier::getCost(const std::string & costName) const
{
  if (costs == nullptr) {
    throw RoadmapExplorerException("Costs map is null");
  }
  auto it = costs->find(costName);
  if (it != costs->end()) {
    return it->second;
  }
  throw RoadmapExplorerException("Cost requested for not a defined field");
}

double Frontier::getWeightedCost() const
{
  if (weighted_cost == nullptr) {
    throw RoadmapExplorerException("Weighted cost frontier property is null");
  }
  return *weighted_cost;
}

bool Frontier::isAchievable() const
{
  if (is_achievable == nullptr) {
    throw RoadmapExplorerException("Is achievable property is null");
  }
  return *is_achievable;
}

bool Frontier::isBlacklisted() const
{
  if (is_blacklisted == nullptr) {
    throw RoadmapExplorerException("Is blacklisted property is null");
  }
  return *is_blacklisted;
}

bool Frontier::isFrontierNull() const
{
  if (goal_point == nullptr) {
    return true;
  }
  return false;
}
