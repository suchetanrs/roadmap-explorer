#include <roadmap_explorer/ExplorationBT.hpp>
namespace roadmap_explorer
{
class InitializationSequence : public BT::StatefulActionNode
{
public:
  InitializationSequence(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<InitCommandVelNode> initialization_controller)
  : BT::StatefulActionNode(name, config)
  {
    initialization_controller_ = initialization_controller;
    LOG_DEBUG("InitializationSequence Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("MODULE InitializationSequence");
    EventLoggerInstance.endEvent("NewIteration", -1);
    LOG_HIGHLIGHT("*******NEW SEQUENCE*******");
    EventLoggerInstance.startEvent("NewIteration");
    if (initialization_controller_->send_velocity_commands()) {
      return BT::NodeStatus::FAILURE;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::RUNNING;
  }

  void onHalted()
  {
    return;
  }
  std::shared_ptr<InitCommandVelNode> initialization_controller_;
};

class WaitForCurrent : public BT::StatefulActionNode
{
public:
  WaitForCurrent(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    rclcpp::Node::SharedPtr ros_node_ptr)
  : BT::StatefulActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    ros_node_ptr_ = ros_node_ptr;
    LOG_DEBUG("WaitForCurrentBT Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("MODULE WaitForCurrent");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!explore_costmap_ros_->isCurrent()) {
      LOG_DEBUG("Waiting for costmap to be current");
      return BT::NodeStatus::RUNNING;
    }
    LOG_DEBUG("CostMap is current.");
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
};

class UpdateBoundaryPolygonBT : public BT::StatefulActionNode
{
public:
  UpdateBoundaryPolygonBT(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<CostAssigner> bel_ptr, std::vector<std::string> boundary_config,
    rclcpp::Node::SharedPtr ros_node_ptr)
  : BT::StatefulActionNode(name, config)
  {
    bel_ptr_ = bel_ptr;
    config_ = boundary_config;
    ros_node_ptr_ = ros_node_ptr;
    LOG_DEBUG("UpdateBoundaryPolygonBT Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("MODULE UpdateBoundaryPolygonBT");
    geometry_msgs::msg::PolygonStamped explore_boundary_;
    geometry_msgs::msg::PointStamped explore_center_;
    explore_boundary_.header.frame_id = "map";
    explore_boundary_.header.stamp = rclcpp::Clock().now();
    for (int i = 0; i < config_.size(); i += 2) {
      geometry_msgs::msg::Point32 point;
      point.x = std::stof(config_[i]);
      point.y = std::stof(config_[i + 1]);
      explore_boundary_.polygon.points.push_back(point);
    }

    explore_center_.header.frame_id = "map";
    explore_center_.point.x = 5.5;
    explore_center_.point.y = 5.5;

    auto updateBoundaryResult = bel_ptr_->updateBoundaryPolygon(explore_boundary_);
    LOG_DEBUG("Adding update boundary polygon for spin.");
    if (updateBoundaryResult == true) {
      LOG_INFO("Region boundary set");
    } else {
      LOG_ERROR("Failed to receive response for updateBoundaryPolygon");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning()
  {
    LOG_DEBUG("UpdateBoundaryPolygonBT On running called ");
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  std::shared_ptr<CostAssigner> bel_ptr_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
  std::vector<std::string> config_;
};

class SearchForFrontiersBT : public BT::StatefulActionNode
{
public:
  SearchForFrontiersBT(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<FrontierSearch> frontierSearchPtr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    rclcpp::Node::SharedPtr ros_node_ptr)
  : BT::StatefulActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    frontierSearchPtr_ = frontierSearchPtr;
    ros_node_ptr_ = ros_node_ptr;
    LOG_DEBUG("SearchForFrontiersBT Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("MODULE SearchForFrontiersBT");
    EventLoggerInstance.startEvent("SearchForFrontiers");
    frontierSearchPtr_->reset();
    explore_costmap_ros_->getCostmap()->getMutex()->lock();
    LOG_DEBUG("SearchForFrontiersBT OnStart called ");
    geometry_msgs::msg::PoseStamped robotP;
    explore_costmap_ros_->getRobotPose(robotP);
    config().blackboard->set<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
    LOG_INFO("Using robotP: " << robotP.pose.position.x << ", " << robotP.pose.position.y);
    auto frontier_list = frontierSearchPtr_->searchFrom(robotP.pose.position);
    auto every_frontier = frontierSearchPtr_->getAllFrontiers();
    if (frontier_list.size() == 0) {
      double increment_value = 0.1;
      getInput("increment_search_distance_by", increment_value);
      frontierSearchPtr_->incrementSearchDistance(increment_value);
      LOG_WARN("No frontiers found in search. Incrementing search radius and returning BT Failure.");
      EventLoggerInstance.endEvent("SearchForFrontiers", 0);
      explore_costmap_ros_->getCostmap()->getMutex()->unlock();
      return BT::NodeStatus::FAILURE;
    }
    LOG_INFO("Recieved " << frontier_list.size() << " frontiers");
    setOutput("frontier_list", frontier_list);
    setOutput("every_frontier", every_frontier);
    RosVisualizer::getInstance().visualizeFrontier(
      frontier_list, every_frontier,
      explore_costmap_ros_->getLayeredCostmap()->getGlobalFrameID());
    frontierSearchPtr_->resetSearchDistance();
    EventLoggerInstance.endEvent("SearchForFrontiers", 0);
    explore_costmap_ros_->getCostmap()->getMutex()->unlock();
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning()
  {
    LOG_INFO("SearchForFrontiersBT On running called ");
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::vector<FrontierPtr>>("frontier_list"),
      BT::OutputPort<std::vector<std::vector<double>>>("every_frontier"),
      BT::InputPort<double>("increment_search_distance_by"),
    };
  }

  std::shared_ptr<FrontierSearch> frontierSearchPtr_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
};

class CleanupRoadMapBT : public BT::StatefulActionNode
{
public:
  CleanupRoadMapBT(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr ros_node_ptr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<FullPathOptimizer> full_path_optimizer)
  : BT::StatefulActionNode(name, config)
  {
    full_path_optimizer_ = full_path_optimizer;
    explore_costmap_ros_ = explore_costmap_ros;
    LOG_INFO("CleanupRoadMapBT Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("MODULE CleanupRoadMapBT");
    // LOG_INFO("Time since last clearance: " << EventLoggerInstance.getTimeSinceStart("clearRoadmap"));
    double time_between_cleanup;
    getInput("time_between_cleanup", time_between_cleanup);
    if (EventLoggerInstance.getTimeSinceStart("clearRoadmap") < time_between_cleanup) {
      return BT::NodeStatus::FAILURE;
    }
    EventLoggerInstance.startEvent("clearRoadmap");
    EventLoggerInstance.startEvent("CleanupRoadMapBT");
    EventLoggerInstance.startEvent("roadmapReconstructionFull");
    bool correct_loop_closure_;
    getInput("correct_loop_closure", correct_loop_closure_);
    LOG_WARN("Reconstructing roadmap and clearing plan cache!");
    FrontierRoadMap::getInstance().reConstructGraph(true, correct_loop_closure_);
    EventLoggerInstance.endEvent("roadmapReconstructionFull", 1);
    full_path_optimizer_->clearPlanCache();
    // TODO: make sure to add a thing such that the entire roadmap within a certain distance (max frontier search distance) is reconstructed periodically
    EventLoggerInstance.endEvent("CleanupRoadMapBT", 0);
    // FrontierRoadMap::getInstance().countTotalItemsInSpatialMap();
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning()
  {
    LOG_INFO("CleanupRoadMapBT On running called ");
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
      BT::InputPort<bool>("correct_loop_closure"),
      BT::InputPort<double>("time_between_cleanup")};
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
};

class UpdateRoadmapBT : public BT::StatefulActionNode
{
public:
  UpdateRoadmapBT(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr ros_node_ptr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
  : BT::StatefulActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    LOG_INFO("UpdateRoadmapBT Constructor");
  }

  BT::NodeStatus onStart() override
  {
    EventLoggerInstance.startEvent("UpdateRoadmapBT");
    LOG_FLOW("MODULE UpdateRoadmapBT");
    std::vector<FrontierPtr> frontier_list;
    getInput<std::vector<FrontierPtr>>("frontier_list", frontier_list);
    LOG_INFO("Recieved " << frontier_list.size() << " frontiers in UpdateRoadmapBT");
    geometry_msgs::msg::PoseStamped robotP;
    if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP)) {
      // Handle the case when "latest_robot_pose" is not found
      LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
      throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
    }
    FrontierRoadMap::getInstance().addNodes(frontier_list, true);
    bool addPose;
    getInput("add_robot_pose_to_roadmap", addPose);
    if (addPose) {
      LOG_FLOW("Adding robot pose as frontier node.");
      FrontierRoadMap::getInstance().addRobotPoseAsNode(robotP.pose, true);
    }
    EventLoggerInstance.startEvent("roadmapReconstruction");
    FrontierRoadMap::getInstance().constructNewEdges(frontier_list);
    FrontierRoadMap::getInstance().constructNewEdgeRobotPose(robotP.pose);
    // FrontierRoadMap::getInstance().reConstructGraph();
    EventLoggerInstance.endEvent("roadmapReconstruction", 1);

    EventLoggerInstance.startEvent("publishRoadmap");
    FrontierRoadMap::getInstance().publishRoadMap();
    EventLoggerInstance.endEvent("publishRoadmap", 2);
    // TODO: make sure to add a thing such that the entire roadmap within a certain distance (max frontier search distance) is reconstructed periodically
    EventLoggerInstance.endEvent("UpdateRoadmapBT", 0);
    // FrontierRoadMap::getInstance().countTotalItemsInSpatialMap();
    // TODO: remove below line
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning()
  {
    LOG_INFO("UpdateRoadmapBT On running called ");
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
      BT::InputPort<bool>("add_robot_pose_to_roadmap")};
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
};

class ProcessFrontierCostsBT : public BT::StatefulActionNode
{
public:
  ProcessFrontierCostsBT(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<CostAssigner> bel_ptr,
    RobotActiveGoals & robot_goals,
    rclcpp::Node::SharedPtr ros_node_ptr)
  : BT::StatefulActionNode(name, config),
    robot_goals_(robot_goals)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    bel_ptr_ = bel_ptr;
    ros_node_ptr_ = ros_node_ptr;
    LOG_INFO("ProcessFrontierCostsBT Constructor");
  }

  BT::NodeStatus onStart() override
  {
    EventLoggerInstance.startEvent("ProcessFrontierCosts");
    LOG_FLOW("MODULE ProcessFrontierCostsBT");
    auto frontierCostsRequestPtr = std::make_shared<GetFrontierCostsRequest>();
    auto frontierCostsResultPtr = std::make_shared<GetFrontierCostsResponse>();

    if (!getInput<std::vector<FrontierPtr>>(
        "frontier_list",
        frontierCostsRequestPtr->frontier_list))
    {
      BT::RuntimeError("No correct input recieved for frontier list");
    }
    if (!getInput<std::vector<std::vector<double>>>(
        "every_frontier",
        frontierCostsRequestPtr->every_frontier))
    {
      BT::RuntimeError("No correct input recieved for every_frontier");
    }
    if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>(
        "latest_robot_pose",
        frontierCostsRequestPtr->start_pose))
    {
      // Handle the case when "latest_robot_pose" is not found
      LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
      throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
    }
    LOG_INFO("Request to get frontier costs sent");
    bool frontierCostsSuccess = bel_ptr_->getFrontierCosts(
      frontierCostsRequestPtr,
      frontierCostsResultPtr);
    if (frontierCostsSuccess == false) {
      LOG_INFO("Failed to receive response for getNextFrontier called from within the robot.");
      EventLoggerInstance.endEvent("ProcessFrontierCosts", 0);
      return BT::NodeStatus::FAILURE;
    }
    setOutput("frontier_costs_result", frontierCostsResultPtr->frontier_list);
    EventLoggerInstance.endEvent("ProcessFrontierCosts", 0);
    RosVisualizer::getInstance().visualizeFrontierMarker(
      frontierCostsResultPtr->frontier_list,
      "map");
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
      BT::InputPort<std::string>("robot_name"),
      BT::InputPort<std::vector<std::vector<double>>>("every_frontier"),
      BT::OutputPort<std::vector<FrontierPtr>>("frontier_costs_result")};
  }

  std::shared_ptr<FrontierSearch> frontierSearchPtr_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  std::shared_ptr<CostAssigner> bel_ptr_;
  RobotActiveGoals & robot_goals_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
};

class OptimizeFullPath : public BT::StatefulActionNode
{
public:
  OptimizeFullPath(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<FullPathOptimizer> full_path_optimizer,
    std::shared_ptr<CostAssigner> bel_ptr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<FrontierSearch> frontierSearchPtr,
    std::shared_ptr<Nav2Interface> nav2_interface,
    std::shared_ptr<RecoveryController> recovery_controller,
    rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    full_path_optimizer_ = full_path_optimizer;
    frontierSearchPtr_ = frontierSearchPtr;
    nav2_interface_ = nav2_interface;
    node_ = node;
    LOG_INFO("OptimizeFullPath Constructor");
  }

  BT::NodeStatus onStart() override
  {
    EventLoggerInstance.startEvent("OptimizeFullPath");
    LOG_FLOW("MODULE OptimizeFullPath");
    std::vector<FrontierPtr> globalFrontierList;
    getInput<std::vector<FrontierPtr>>("frontier_costs_result", globalFrontierList);
    geometry_msgs::msg::PoseStamped robotP;
    geometry_msgs::msg::PoseStamped robotP3D;
    if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP)) {
      // Handle the case when "latest_robot_pose" is not found
      LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
      throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
    }
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    auto return_state = full_path_optimizer_->getNextGoal(
      globalFrontierList, allocatedFrontier, 3,
      robotP);
    if (return_state) {
      frontierSearchPtr_->resetSearchDistance();
      setOutput<FrontierPtr>("allocated_frontier", allocatedFrontier);
    } else {
      nav2_interface_->cancelAllGoals();
      double increment_value = 0.1;
      getInput("increment_search_distance_by", increment_value);
      frontierSearchPtr_->incrementSearchDistance(increment_value);
      recovery_controller_->computeVelocityCommand(false);
      EventLoggerInstance.endEvent("OptimizeFullPath", 0);
      return BT::NodeStatus::FAILURE;
    }
    frontierSearchPtr_->resetSearchDistance();
    EventLoggerInstance.endEvent("OptimizeFullPath", 0);
    setOutput<FrontierPtr>("allocated_frontier", allocatedFrontier);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<FrontierPtr>>("frontier_costs_result"),
      BT::InputPort<bool>("use_fisher_information"),
      BT::OutputPort<FrontierPtr>("allocated_frontier"),
      BT::InputPort<double>("increment_search_distance_by"),
      BT::InputPort<double>("number_retries_fi")};
  }

  std::shared_ptr<FrontierSearch> frontierSearchPtr_;
  std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  std::shared_ptr<Nav2Interface> nav2_interface_;
  std::shared_ptr<RecoveryController> recovery_controller_;
  rclcpp::Node::SharedPtr node_;
};

class HysterisisControl : public BT::StatefulActionNode
{
public:
  HysterisisControl(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<FullPathOptimizer> full_path_optimizer)
  : BT::StatefulActionNode(name, config)
  {
    LOG_INFO("HysterisisControl Constructor");
    minDistance = std::numeric_limits<double>::max();
    EventLoggerInstance.startEvent("startHysterisis");
    full_path_optimizer_ = full_path_optimizer;
  }

  BT::NodeStatus onStart() override
  {
    EventLoggerInstance.startEvent("HysterisisControl");
    LOG_FLOW("MODULE HysterisisControl");
    geometry_msgs::msg::PoseStamped robotP;
    if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP)) {
      // Handle the case when "latest_robot_pose" is not found
      LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
      throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
    }
    CurrentGoalStatus status;
    if (!config().blackboard->get<CurrentGoalStatus>("current_goal_status", status)) {
      LOG_FATAL("Failed to retrieve hysteresis_enabled from blackboard.");
      throw std::runtime_error("Failed to retrieve hysteresis_enabled from blackboard.");
    }
    /**
         * The status is set to RUNNING when when the robot is still replanning (typically at 1Hz) towards a goal.
         * The status is set to SUCCESS when the robot has reached the goal / the goal is mapped.
         */
    if (status == CurrentGoalStatus::RUNNING) {
      FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
      getInput<FrontierPtr>("allocated_frontier", allocatedFrontier);
      LOG_INFO("Hysterisis prior: " << allocatedFrontier);

      LOG_DEBUG("Value of min Distance " << minDistance);
      LOG_DEBUG("Reference of min Distance " << &minDistance);
      if (parameterInstance.getValue<bool>("goalHysteresis.use_euclidean_distance") == true) {
        LOG_INFO("Using Euclidean Distance for hysteresis");
        if (distanceBetweenPoints(
            allocatedFrontier->getGoalPoint(),
            robotP.pose.position) < minDistance - 3.0)
        {
          minDistance = distanceBetweenPoints(
            allocatedFrontier->getGoalPoint(), robotP.pose.position);
          mostFrequentFrontier = allocatedFrontier;
          LOG_DEBUG("Found a closer one: " << minDistance);
        }
      } else if (parameterInstance.getValue<bool>("goalHysteresis.use_roadmap_planner_distance") ==
        true)
      {
        FrontierPtr robotPoseFrontier = std::make_shared<Frontier>();
        robotPoseFrontier->setGoalPoint(robotP.pose.position.x, robotP.pose.position.y);
        robotPoseFrontier->setUID(generateUID(robotPoseFrontier));
        robotPoseFrontier->setPathLength(0.0);
        robotPoseFrontier->setPathLengthInM(0.0);
        LOG_INFO("Using Roadmap Planner Distance for hysteresis");
        auto lengthToGoal = full_path_optimizer_->calculateLengthRobotToGoal(
          robotPoseFrontier,
          allocatedFrontier,
          robotP);
        if (lengthToGoal < minDistance - 3.0) {
          minDistance = lengthToGoal;
          mostFrequentFrontier = allocatedFrontier;
          LOG_DEBUG("Found a closer one: " << minDistance);
        }
      }

      // Set the most frequent frontier as the output
      setOutput<FrontierPtr>("allocated_frontier_after_hysterisis", mostFrequentFrontier);
    } else {
      FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
      getInput<FrontierPtr>("allocated_frontier", allocatedFrontier);
      LOG_INFO("Hysterisis prior: " << allocatedFrontier);
      setOutput<FrontierPtr>("allocated_frontier_after_hysterisis", allocatedFrontier);
      if (parameterInstance.getValue<bool>("goalHysteresis.use_euclidean_distance") == true) {
        minDistance =
          distanceBetweenPoints(allocatedFrontier->getGoalPoint(), robotP.pose.position);
      } else if (parameterInstance.getValue<bool>("goalHysteresis.use_roadmap_planner_distance") ==
        true)
      {
        FrontierPtr robotPoseFrontier = std::make_shared<Frontier>();
        robotPoseFrontier->setGoalPoint(robotP.pose.position.x, robotP.pose.position.y);
        robotPoseFrontier->setUID(generateUID(robotPoseFrontier));
        robotPoseFrontier->setPathLength(0.0);
        robotPoseFrontier->setPathLengthInM(0.0);
        minDistance = full_path_optimizer_->calculateLengthRobotToGoal(
          robotPoseFrontier,
          allocatedFrontier, robotP);
      }
      LOG_INFO("Current length to goal: " << minDistance);
      mostFrequentFrontier = allocatedFrontier;
    }
    LOG_INFO("Hysterisis post: " << mostFrequentFrontier);
    EventLoggerInstance.endEvent("HysterisisControl", 0);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<FrontierPtr>("allocated_frontier"),
      BT::OutputPort<FrontierPtr>("allocated_frontier_after_hysterisis")};
  }

  double minDistance;
  FrontierPtr mostFrequentFrontier = std::make_shared<Frontier>();
  std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
};

class ExecuteRecoveryMove : public BT::StatefulActionNode
{
public:
  ExecuteRecoveryMove(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RecoveryController> recovery_controller)
  : BT::StatefulActionNode(name, config)
  {
    recovery_controller_bt_ = recovery_controller;
    LOG_INFO("ExecuteRecoveryMove Constructor");
  }

  BT::NodeStatus onStart() override
  {
    EventLoggerInstance.startEvent("ExecuteRecoveryMove");
    LOG_FLOW("MODULE ExecuteRecoveryMove");
    if (!recovery_controller_bt_) {
      throw std::runtime_error("recovery ptr is null.");
    }
    bool backwardOnly;
    getInput<bool>("backward_only", backwardOnly);
    recovery_controller_bt_->computeVelocityCommand(backwardOnly);
    EventLoggerInstance.endEvent("ExecuteRecoveryMove", 0);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("backward_only")};
  }

  std::shared_ptr<RecoveryController> recovery_controller_bt_;
};

class SendNav2Goal : public BT::StatefulActionNode
{
public:
  SendNav2Goal(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<Nav2Interface> nav2_interface,
    rclcpp::Node::SharedPtr ros_node_ptr)
  : BT::StatefulActionNode(name, config)
  {
    nav2_interface_ = nav2_interface;
    ros_node_ptr_ = ros_node_ptr;
    latestAllocationFailures_ = 0;
    LOG_INFO("SendNav2Goal Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("SendNav2Goal onStart");
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    latestAllocation_ = allocatedFrontier;
    getInput("allocated_frontier", allocatedFrontier);
    // LOG_INFO("Allocated frontier oz:" + std::to_string(allocatedFrontier->getGoalOrientation().z));
    // LOG_INFO("Allocated frontier ow:" + std::to_string(allocatedFrontier->getGoalOrientation().w));
    // LOG_INFO("Allocated frontier ox:" + std::to_string(allocatedFrontier->getGoalOrientation().x));
    // LOG_INFO("Allocated frontier oy:" + std::to_string(allocatedFrontier->getGoalOrientation().y));
    // LOG_INFO("Allocated frontier uid:" + std::to_string(allocatedFrontier->getUID()));
    geometry_msgs::msg::PoseStamped goalPose;
    goalPose.header.frame_id = "map";
    goalPose.pose.position = allocatedFrontier->getGoalPoint();
    goalPose.pose.orientation = allocatedFrontier->getGoalOrientation();
    nav2_interface_->sendGoal(goalPose);
    EventLoggerInstance.startEvent("GoalSentToNav2");
    time_for_planning_ = EventLoggerInstance.getTimeSinceStart("TimeForPlanning");
    LOG_INFO("Planning time: " << time_for_planning_);
    EventLoggerInstance.startEvent("SendNav2GoalModule");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning()
  {
    double timeoutValue;
    getInput("timeout_value", timeoutValue);
    LOG_DEBUG("SendNav2Goal onRunning");
    if (nav2_interface_->goalStatus() == 0 && EventLoggerInstance.getTimeSinceStart(
        "GoalSentToNav2") + time_for_planning_ < timeoutValue)
    {
      return BT::NodeStatus::RUNNING;
    } else if (nav2_interface_->goalStatus() == 0 &&
      EventLoggerInstance.getTimeSinceStart("GoalSentToNav2") + time_for_planning_ >=
      timeoutValue)
    {
      EventLoggerInstance.startEvent("TimeForPlanning");
      EventLoggerInstance.endEvent("SendNav2GoalModule", 0);
      return BT::NodeStatus::SUCCESS;
    } else if (nav2_interface_->goalStatus() == 1) {
      EventLoggerInstance.startEvent("TimeForPlanning");
      EventLoggerInstance.endEvent("SendNav2GoalModule", 0);
      latestAllocationFailures_ = 0;
      return BT::NodeStatus::SUCCESS;
    } else if (nav2_interface_->goalStatus() == -1) {
      latestAllocationFailures_++;
      if (latestAllocationFailures_ > 4) {
        latestAllocationFailures_ = 0;
        latestAllocation_->setBlacklisted(true);
      }
      EventLoggerInstance.startEvent("TimeForPlanning");
      EventLoggerInstance.endEvent("SendNav2GoalModule", 0);
      return BT::NodeStatus::FAILURE;
    } else {
      throw BT::RuntimeError("Unknown goal status");
    }
    EventLoggerInstance.endEvent("SendNav2GoalModule", 0);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<FrontierPtr>("allocated_frontier"),
      BT::InputPort<double>("timeout_value")};
  }

  std::shared_ptr<Nav2Interface> nav2_interface_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
  FrontierPtr latestAllocation_ = std::make_shared<Frontier>();
  int latestAllocationFailures_;
  double time_for_planning_;
};

class CheckIfGoalMapped : public BT::StatefulActionNode
{
public:
  CheckIfGoalMapped(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    rclcpp::Node::SharedPtr ros_node_ptr,
    std::shared_ptr<FullPathOptimizer> full_path_optimizer)
  : BT::StatefulActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    ros_node_ptr_ = ros_node_ptr;
    full_path_optimizer_ = full_path_optimizer;
    LOG_INFO("CheckIfGoalMapped Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("CheckIfGoalMapped onStart");
    EventLoggerInstance.startEvent("CheckIfGoalMapped");
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    getInput("allocated_frontier", allocatedFrontier);
    // LOG_INFO("Allocated frontier oz:" + std::to_string(allocatedFrontier->getGoalOrientation().z));
    // LOG_INFO("Allocated frontier ow:" + std::to_string(allocatedFrontier->getGoalOrientation().w));
    // LOG_INFO("Allocated frontier ox:" + std::to_string(allocatedFrontier->getGoalOrientation().x));
    // LOG_INFO("Allocated frontier oy:" + std::to_string(allocatedFrontier->getGoalOrientation().y));
    // LOG_INFO("Allocated frontier uid:" + std::to_string(allocatedFrontier->getUID()));
    geometry_msgs::msg::PoseStamped goalPose;
    goalPose.header.frame_id = "map";
    goalPose.pose.position = allocatedFrontier->getGoalPoint();
    if (surroundingCellsMapped(goalPose.pose.position, *explore_costmap_ros_->getCostmap())) {
      LOG_WARN("Goal is mapped. Restarting....");
      config().blackboard->set<CurrentGoalStatus>(
        "current_goal_status",
        CurrentGoalStatus::COMPLETE);
      EventLoggerInstance.endEvent("CheckIfGoalMapped", 0);
      return BT::NodeStatus::SUCCESS;
    }
    geometry_msgs::msg::PoseStamped robotP;
    if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP)) {
      // Handle the case when "latest_robot_pose" is not found
      LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
      throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
    }
    if (!full_path_optimizer_->refineAndPublishPath(robotP, allocatedFrontier)) {
      LOG_ERROR(
        "Failed to refine and publish path between robotP: " << robotP.pose.position.x << ", " << robotP.pose.position.y << " and " <<
          allocatedFrontier);
      config().blackboard->set<CurrentGoalStatus>(
        "current_goal_status",
        CurrentGoalStatus::COMPLETE);
      EventLoggerInstance.endEvent("CheckIfGoalMapped", 0);
      return BT::NodeStatus::SUCCESS;
    }
    EventLoggerInstance.endEvent("CheckIfGoalMapped", 0);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onRunning()
  {
    throw BT::RuntimeError("ReplanTimeoutCompleteBT should not be running");
    return BT::NodeStatus::FAILURE;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<FrontierPtr>("allocated_frontier")};
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
  std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
};

class ReplanTimeoutCompleteBT : public BT::StatefulActionNode
{
public:
  ReplanTimeoutCompleteBT(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    rclcpp::Node::SharedPtr ros_node_ptr)
  : BT::StatefulActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    ros_node_ptr_ = ros_node_ptr;
    LOG_INFO("ReplanTimeoutCompleteBT Constructor");
  }

  BT::NodeStatus onStart() override
  {
    double timeoutValue;
    getInput("timeout_value", timeoutValue);
    LOG_FLOW("ReplanTimeoutCompleteBT onStart");
    if (EventLoggerInstance.getTimeSinceStart("triggeredReplan") > timeoutValue) {
      LOG_WARN("Replan duration exceeded. Check cpu performance!!");
      EventLoggerInstance.startEvent("triggeredReplan");
      return BT::NodeStatus::FAILURE;
    }
    if (EventLoggerInstance.getTimeSinceStart("replanTimeout") > timeoutValue) {
      LOG_WARN("Replanning timed out. Restarting the frontier computation");
      EventLoggerInstance.startEvent("replanTimeout");
      config().blackboard->set<CurrentGoalStatus>(
        "current_goal_status",
        CurrentGoalStatus::RUNNING);
      EventLoggerInstance.startEvent("triggeredReplan");
      return BT::NodeStatus::SUCCESS;
    }
    EventLoggerInstance.startEvent("triggeredReplan");
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<FrontierPtr>("allocated_frontier"),
      BT::InputPort<double>("timeout_value")};
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
};

class RecoveryMoveBack : public BT::StatefulActionNode
{
public:
  RecoveryMoveBack(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    rclcpp::Node::SharedPtr ros_node_ptr)
  : BT::StatefulActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    ros_node_ptr_ = ros_node_ptr;
    cmd_vel_publisher_ =
      ros_node_ptr->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
    LOG_INFO("RecoveryMoveBack Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("RecoveryMoveBack onStart");
    startTime = std::chrono::high_resolution_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning()
  {
    LOG_INFO("RecoveryMoveBack onRunning");
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = endTime - startTime;
    double maxDuration;
    getInput("move_back_duration", maxDuration);
    if (duration.count() > maxDuration) {
      geometry_msgs::msg::Twist twist_msg;
      cmd_vel_publisher_->publish(twist_msg);
      cmd_vel_publisher_->publish(twist_msg);
      cmd_vel_publisher_->publish(twist_msg);
      return BT::NodeStatus::SUCCESS;
    }
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = -0.5;         // -0.5 m/s for reverse motion
    cmd_vel_publisher_->publish(twist_msg);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted()
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("move_back_duration")};
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};
}

namespace roadmap_explorer
{
FrontierExplorationServer::FrontierExplorationServer(rclcpp::Node::SharedPtr node)
{
  LOG_TRACE("First BT constructor");
  bt_node_ = node;
  exploration_active_ = true;
  blackboard = BT::Blackboard::create();
  blackboard->set<CurrentGoalStatus>("current_goal_status", CurrentGoalStatus::RUNNING);

  robot_namespaces_ = {};
  bt_node_->declare_parameter("robot_namespaces", rclcpp::ParameterValue(robot_namespaces_));
  config_ = {"100.0", "100.0", "100.0", "-100.0", "-100.0", "-100.0", "-100.0", "100.0"};
  bt_node_->declare_parameter("config", rclcpp::ParameterValue(config_));
  bt_node_->declare_parameter("process_other_robots", rclcpp::ParameterValue(false));
  bt_node_->declare_parameter(
    "bt_xml_path",
    "/root/dev_ws/src/roadmap_explorer/roadmap_explorer/xml/exploration.xml");

  bt_node_->get_parameter("robot_namespaces", robot_namespaces_);
  bt_node_->get_parameter("config", config_);
  bt_node_->get_parameter("process_other_robots", process_other_robots_);
  bt_node_->get_parameter("bt_xml_path", bt_xml_path_);
  parameterInstance.makeParameters(true, node);
  LOG_TRACE("Declared BT params");
  // //--------------------------------------------NAV2 CLIENT RELATED-------------------------------
  // nav2_interface_ = std::make_shared<Nav2Interface>(bt_node_);
  // LOG_TRACE("Created Nav2 interface instance");

  //--------------------------------------------EXPLORE SERVER RELATED----------------------------

  explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "roadmap_explorer_costmap", "", "roadmap_explorer_costmap");
  explore_costmap_ros_->configure();
  // Launch a thread to run the costmap node
  explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
  explore_costmap_ros_->activate();
  LOG_TRACE("Created costmap instance");

  recovery_controller_ = std::make_shared<RecoveryController>(explore_costmap_ros_, bt_node_);
  initialization_controller_ = std::make_shared<InitCommandVelNode>();

  //------------------------------------------BOUNDED EXPLORE LAYER RELATED------------------------
  RosVisualizer::createInstance(bt_node_, explore_costmap_ros_->getCostmap());
  FrontierRoadMap::createInstance(explore_costmap_ros_);
  LOG_TRACE("Created ros visualizer instance");
  bel_ptr_ = std::make_shared<CostAssigner>(explore_costmap_ros_);

  frontierSearchPtr_ =
    std::make_shared<FrontierSearch>(*(explore_costmap_ros_->getLayeredCostmap()->getCostmap()));
  full_path_optimizer_ = std::make_shared<FullPathOptimizer>(bt_node_, explore_costmap_ros_);
  //---------------------------------------------ROS RELATED------------------------------------------
  rviz_control_callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = rviz_control_callback_group_;

  exploration_rviz_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "/exploration_state", 10, std::bind(
      &FrontierExplorationServer::rvizControl, this,
      std::placeholders::_1),
    options);
  LOG_INFO("FrontierExplorationServer::FrontierExplorationServer()");
}

FrontierExplorationServer::~FrontierExplorationServer()
{
  LOG_INFO("FrontierExplorationServer::~FrontierExplorationServer()");
  explore_costmap_ros_->deactivate();
  explore_costmap_ros_->cleanup();
  explore_costmap_thread_.reset();
}

void FrontierExplorationServer::makeBTNodes()
{
  while (!explore_costmap_ros_->isCurrent()) {
    LOG_WARN("Waiting for explore costmap to be current.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  EventLoggerInstance.startEvent("clearRoadmap");
  EventLoggerInstance.startEvent("replanTimeout");

  BT::NodeBuilder builder_initialization =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<InitializationSequence>(name, config, initialization_controller_);
    };
  factory.registerBuilder<WaitForCurrent>("InitializationSequence", builder_initialization);

  BT::NodeBuilder builder_wait_for_current =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<WaitForCurrent>(name, config, explore_costmap_ros_, bt_node_);
    };
  factory.registerBuilder<WaitForCurrent>("WaitForCurrent", builder_wait_for_current);

  BT::NodeBuilder builder_update_boundary_polygon =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<UpdateBoundaryPolygonBT>(name, config, bel_ptr_, config_, bt_node_);
    };
  factory.registerBuilder<UpdateBoundaryPolygonBT>(
    "UpdateBoundaryPolygon",
    builder_update_boundary_polygon);

  BT::NodeBuilder builder_frontier_search =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<SearchForFrontiersBT>(
        name, config, frontierSearchPtr_,
        explore_costmap_ros_, bt_node_);
    };
  factory.registerBuilder<SearchForFrontiersBT>("SearchForFrontiers", builder_frontier_search);

  BT::NodeBuilder builder_update_roadmap_data =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<UpdateRoadmapBT>(name, config, bt_node_, explore_costmap_ros_);
    };
  factory.registerBuilder<UpdateRoadmapBT>("UpdateFrontierRoadmap", builder_update_roadmap_data);

  BT::NodeBuilder builder_cleanup_roadmap_data =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<CleanupRoadMapBT>(
        name, config, bt_node_, explore_costmap_ros_,
        full_path_optimizer_);
    };
  factory.registerBuilder<CleanupRoadMapBT>("CleanupRoadmap", builder_cleanup_roadmap_data);

  BT::NodeBuilder builder_frontier_costs =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<ProcessFrontierCostsBT>(
        name, config, explore_costmap_ros_, bel_ptr_,
        robot_active_goals_, bt_node_);
    };
  factory.registerBuilder<ProcessFrontierCostsBT>("ProcessFrontierCosts", builder_frontier_costs);

  BT::NodeBuilder builder_full_path_optimizer =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<OptimizeFullPath>(
        name, config, full_path_optimizer_, bel_ptr_,
        explore_costmap_ros_, frontierSearchPtr_,
        nav2_interface_, recovery_controller_, bt_node_);
    };
  factory.registerBuilder<OptimizeFullPath>("OptimizeFullPath", builder_full_path_optimizer);

  BT::NodeBuilder builder_hystersis_control =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<HysterisisControl>(name, config, full_path_optimizer_);
    };
  factory.registerBuilder<HysterisisControl>("HysterisisControl", builder_hystersis_control);

  BT::NodeBuilder builder_send_goal =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<SendNav2Goal>(name, config, nav2_interface_, bt_node_);
    };
  factory.registerBuilder<SendNav2Goal>("SendNav2Goal", builder_send_goal);

  BT::NodeBuilder builder_goal_mapped =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<CheckIfGoalMapped>(
        name, config, explore_costmap_ros_, bt_node_,
        full_path_optimizer_);
    };
  factory.registerBuilder<CheckIfGoalMapped>("CheckIfGoalMapped", builder_goal_mapped);

  BT::NodeBuilder builder_replan_timeout =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<ReplanTimeoutCompleteBT>(
        name, config, explore_costmap_ros_,
        bt_node_);
    };
  factory.registerBuilder<ReplanTimeoutCompleteBT>("ReplanTimeoutComplete", builder_replan_timeout);

  BT::NodeBuilder builder_recovery_back =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<RecoveryMoveBack>(name, config, explore_costmap_ros_, bt_node_);
    };
  factory.registerBuilder<RecoveryMoveBack>("RecoveryMoveBack", builder_recovery_back);

  BT::NodeBuilder execute_recovery_move =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<ExecuteRecoveryMove>(name, config, recovery_controller_);
    };
  factory.registerBuilder<ExecuteRecoveryMove>("ExecuteRecoveryMove", execute_recovery_move);

  behaviour_tree = factory.createTreeFromFile(bt_xml_path_, blackboard);
}

void FrontierExplorationServer::run()
{
  int bt_sleep_duration = parameterInstance.getValue<int64_t>("explorationBT.bt_sleep_ms");
  while (rclcpp::ok()) {
    if (exploration_active_) {
      behaviour_tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(bt_sleep_duration));
      LOG_DEBUG("TICKED ONCE");
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void FrontierExplorationServer::rvizControl(std_msgs::msg::Int32 rvizControlValue)
{
  if (rvizControlValue.data == 0) {
    LOG_WARN("Pausing exploration");
    exploration_active_ = false;
    nav2_interface_->cancelAllGoals();
  } else if (rvizControlValue.data == 1) {
    LOG_WARN("Playing exploration");
    exploration_active_ = true;
  }
}

}
