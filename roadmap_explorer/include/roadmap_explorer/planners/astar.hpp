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

  Node();

  Node(FrontierPtr frontier_in, double g, double h, std::shared_ptr<Node> parent = nullptr)
  : frontier(frontier_in), g(g), h(h), f(g + h), parent(parent) {}

  // Comparator for priority queue to order by f value
  bool operator>(const Node & other) const
  {
    return f > other.f;
  }
};

struct fCostNodeCompare
{
  bool operator()(const std::shared_ptr<Node> & a, const std::shared_ptr<Node> & b) const
  {
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
    std::unordered_map<FrontierPtr,
    std::vector<FrontierPtr>,
    FrontierHash> & roadmap_);

protected:
  double heuristic(const Node & a, const Node & b);

  double heuristic(std::shared_ptr<Node> a, std::shared_ptr<Node> b);

  double heuristic(const FrontierPtr & a, const FrontierPtr & b);

  std::vector<std::shared_ptr<Node>> getSuccessors(
    std::shared_ptr<Node> current,
    std::shared_ptr<Node> goal,
    std::unordered_map<FrontierPtr,
    std::vector<FrontierPtr>,
    FrontierHash> & roadmap_);

};
