#include "roadmap_explorer/planners/Astar.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"

FrontierRoadmapAStar::FrontierRoadmapAStar()
{
}

FrontierRoadmapAStar::~FrontierRoadmapAStar()
{
  LOG_INFO("FrontierRoadmapAStar::~FrontierRoadmapAStar()");
}

// Function to calculate heuristic (Euclidean distance in this case)
double FrontierRoadmapAStar::heuristic(const Node & a, const Node & b)
{
  return (a.frontier->getGoalPoint().x - b.frontier->getGoalPoint().x) *
         (a.frontier->getGoalPoint().x - b.frontier->getGoalPoint().x) +
         (a.frontier->getGoalPoint().y - b.frontier->getGoalPoint().y) *
         (a.frontier->getGoalPoint().y - b.frontier->getGoalPoint().y);
}

double FrontierRoadmapAStar::heuristic(
  const std::shared_ptr<Node> & a,
  const std::shared_ptr<Node> & b)
{
  return (a->frontier->getGoalPoint().x - b->frontier->getGoalPoint().x) *
         (a->frontier->getGoalPoint().x - b->frontier->getGoalPoint().x) +
         (a->frontier->getGoalPoint().y - b->frontier->getGoalPoint().y) *
         (a->frontier->getGoalPoint().y - b->frontier->getGoalPoint().y);
}

double FrontierRoadmapAStar::heuristic(const FrontierPtr & a, const FrontierPtr & b)
{
  return (a->getGoalPoint().x - b->getGoalPoint().x) * (a->getGoalPoint().x - b->getGoalPoint().x) +
         (a->getGoalPoint().y - b->getGoalPoint().y) * (a->getGoalPoint().y - b->getGoalPoint().y);
}

// Function to get the successors of a node (example for a grid)
std::vector<std::shared_ptr<Node>> FrontierRoadmapAStar::getSuccessors(
  const std::shared_ptr<Node> & current, const std::shared_ptr<Node> & goal,
  const std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> & roadmap)
{
  // LOG_TRACE("Successor size: " << roadmap_[current->frontier].size());
  std::vector<std::shared_ptr<Node>> successors;
  if (roadmap.count(current->frontier) == 0) {
    LOG_FATAL(current->frontier << " not present.");
    throw RoadmapExplorerException("This should never happen. FrontierPtr not found in roadmap.");
  }

  for (const auto & dir : roadmap.at(current->frontier)) {
    double newG = current->g + roadmap_explorer::sqDistanceBetweenFrontiers(current->frontier, dir);     // Assuming uniform cost for each move
    double newH = heuristic(dir, goal->frontier);
    auto newNode = std::make_shared<Node>(dir, newG, newH);
    successors.push_back(newNode);
  }
  return successors;
}

// A* Algorithm function
std::pair<std::vector<std::shared_ptr<Node>>, double> FrontierRoadmapAStar::getPlan(
  const FrontierPtr & start, const FrontierPtr & goal, const std::unordered_map<FrontierPtr,
  std::vector<FrontierPtr>, FrontierHash> & roadmap)
{
  auto start_ = std::make_shared<Node>(start, 0.0, heuristic(start, goal));
  auto goal_ = std::make_shared<Node>(goal, 0.0, 0.0);
  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>,
    FCostNodeCompare> openList;                                                                                // Min-heap priority queue
  std::unordered_set<int> closedSet;
  std::unordered_map<int, std::shared_ptr<Node>> allNodes;   // To store all nodes and their costs

  openList.push(start_);
  allNodes[start_->frontier->getUID()] = start_;

  while (!openList.empty() && rclcpp::ok()) {
    std::shared_ptr<Node> current = openList.top();
    openList.pop();

    // If goal is reached (use frontier UID comparison for exact match)
    if (current->frontier->getUID() == goal_->frontier->getUID()) {
      std::vector<std::shared_ptr<Node>> path;
      std::shared_ptr<Node> node = allNodes[current->frontier->getUID()];
      double total_path_length = 0;
      while (node != nullptr && rclcpp::ok()) {
        path.push_back(node);
        if (path.size() > 1) {
          total_path_length += sqrt(heuristic(path[path.size() - 2], node));
        }
        node = node->parent;
      }
      reverse(path.begin(), path.end());
      return std::make_pair(path, total_path_length);
    }

    closedSet.insert(current->frontier->getUID());

    // Generate successors
    std::vector<std::shared_ptr<Node>> successors = getSuccessors(current, goal_, roadmap);

    for (auto & successor : successors) {
      int successorHash = successor->frontier->getUID();

      if (closedSet.find(successorHash) != closedSet.end()) {
        continue;
      }

      if (allNodes.find(successorHash) == allNodes.end() ||
        allNodes[successorHash]->g > successor->g)
      {
        successor->parent = allNodes[current->frontier->getUID()];
        allNodes[successorHash] = successor;
        openList.push(successor);
      }
    }
  }

  // If no path found, return an empty path
  return std::make_pair(std::vector<std::shared_ptr<Node>>(), 0);
}

// int main() {
//     // Define the grid (0 = empty, 1 = obstacle)
//     std::vector<std::vector<int>> grid = {
//         {0, 1, 0, 0, 0},
//         {0, 1, 0, 1, 0},
//         {0, 0, 0, 0, 0},
//         {0, 1, 0, 1, 1},
//         {0, 0, 0, 0, 0}
//     };

//     Node start(0, 0, 0, 0); // Start node (x, y, g, h)
//     Node goal(4, 4, 0, 0); // Goal node (x, y, g, h)

//     std::vector<Node> path = aStar(start, goal, grid);

//     // Print the path
//     if (!path.empty()) {
//         cout << "Path found:" << endl;
//         for (auto& node : path) {
//             cout << "(" << node.x << ", " << node.y << ") ";
//         }
//         cout << endl;
//     } else {
//         cout << "No path found." << endl;
//     }

//     return 0;
// }
