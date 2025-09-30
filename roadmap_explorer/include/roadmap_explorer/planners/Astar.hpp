#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <functional>

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Helpers.hpp"


// Define a Node structure
struct Node
{
  FrontierPtr frontier;
  double g;   // Cost from the start node to this node
  double h;   // Heuristic cost estimate to the goal
  double f;   // Total cost (f = g + h)
  std::shared_ptr<Node> parent;   // Pointer to the parent node

  Node()
  : frontier(nullptr), g(0.0), h(0.0), f(0.0), parent(nullptr) {}

  Node(
    FrontierPtr frontier_in, double g_cost, double h_cost,
    std::shared_ptr<Node> parent_node = nullptr)
  : frontier(frontier_in), g(g_cost), h(h_cost), f(g_cost + h_cost), parent(parent_node) {}

  // Comparator for priority queue to order by f value
  bool operator>(const Node & other) const
  {
    return f > other.f;
  }
};

struct FCostNodeCompare
{
  bool operator()(const std::shared_ptr<Node> & a, const std::shared_ptr<Node> & b) const
  {
    if (std::abs(a->f - b->f) < 1e-9) {
      // If f-costs are equal, prefer lower h-cost (closer to goal)
      return a->h > b->h;
    }
    return a->f > b->f;
  }
};

class FrontierRoadmapAStar
{
public:
  FrontierRoadmapAStar();

  ~FrontierRoadmapAStar();

  std::pair<std::vector<std::shared_ptr<Node>>, double> getPlan(
    const FrontierPtr & start,
    const FrontierPtr & goal,
    const std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> & roadmap);

protected:
  double heuristic(const Node & a, const Node & b);

  double heuristic(const std::shared_ptr<Node> & a, const std::shared_ptr<Node> & b);

  double heuristic(const FrontierPtr & a, const FrontierPtr & b);

  std::vector<std::shared_ptr<Node>> getSuccessors(
    const std::shared_ptr<Node> & current,
    const std::shared_ptr<Node> & goal,
    const std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> & roadmap);

};

#endif  // ASTAR_HPP_
