#include <iostream>
#include <roadmap_explorer/Frontier.hpp>
#include <geometry_msgs/msg/point.hpp>

int main()
{
  // Create two instances of the FrontierPtr class
  FrontierPtr frontier1;
  FrontierPtr frontier2;

  // Set the same goal point in both frontiers
  geometry_msgs::msg::Point goalPoint;
  goalPoint.x = 1.0;
  goalPoint.y = 2.0;
  goalPoint.z = 0.0;

  frontier1->setGoalPoint(goalPoint);
  frontier2->setGoalPoint(goalPoint);

  // Check if the two frontiers are equal using the == operator
  if (frontier1 == frontier2) {
    std::cout << "The two frontiers are equal." << std::endl;
    auto h1 = std::hash<double>()(frontier1->getGoalPoint().x) ^
      (std::hash<double>()(frontier1->getGoalPoint().y) << 1);
    auto h2 = std::hash<double>()(frontier2->getGoalPoint().x) ^
      (std::hash<double>()(frontier2->getGoalPoint().y) << 1);
    std::cout << "F1 hash:" << h1 << std::endl;
    std::cout << "F1 hash:" << h2 << std::endl;
    std::cout << "The two frontiers are equal." << std::endl;
  } else {
    std::cout << "The two frontiers are not equal." << std::endl;
  }

  return 0;
}
