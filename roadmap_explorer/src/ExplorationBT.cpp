#include <roadmap_explorer/ExplorationBT.hpp>
namespace roadmap_explorer
{

class UpdateBoundaryPolygonBT : public BT::SyncActionNode
{
public:
  UpdateBoundaryPolygonBT(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<CostAssigner> cost_assigner_ptr,
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
  : BT::SyncActionNode(name, config)
  {
    cost_assigner_ptr_ = cost_assigner_ptr;
    config_ = parameterInstance.getValue<std::vector<double>>("explorationBT.exploration_boundary");
    ros_node_ptr_ = ros_node_ptr;
    LOG_DEBUG("UpdateBoundaryPolygonBT Constructor");
  }

  BT::NodeStatus tick() override
  {
    LOG_FLOW("MODULE UpdateBoundaryPolygonBT");
    geometry_msgs::msg::PolygonStamped explore_boundary_;
    explore_boundary_.header.frame_id = "map";
    explore_boundary_.header.stamp = rclcpp::Clock().now();
    for (int i = 0; i < (int)config_.size(); i += 2) {
      geometry_msgs::msg::Point32 point;
      point.x = config_[i];
      point.y = config_[i + 1];
      LOG_DEBUG("Adding point to boundary: " << point.x << ", " << point.y);
      explore_boundary_.polygon.points.push_back(point);
    }

    cost_assigner_ptr_->updateBoundaryPolygon(explore_boundary_);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  std::shared_ptr<CostAssigner> cost_assigner_ptr_;
  std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
  std::vector<double> config_;
};

class SearchForFrontiersBT : public BT::SyncActionNode
{
public:
  SearchForFrontiersBT(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<FrontierSearch> frontierSearchPtr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
  : BT::SyncActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    frontierSearchPtr_ = frontierSearchPtr;
    ros_node_ptr_ = ros_node_ptr;
    LOG_DEBUG("SearchForFrontiersBT Constructor");
  }

  BT::NodeStatus tick() override
  {
    LOG_FLOW("MODULE SearchForFrontiersBT");
    EventLoggerInstance.startEvent("SearchForFrontiers");
    frontierSearchPtr_->reset();
    explore_costmap_ros_->getCostmap()->getMutex()->lock();
    LOG_DEBUG("SearchForFrontiersBT OnStart called ");
    geometry_msgs::msg::PoseStamped robotP;
    explore_costmap_ros_->getRobotPose(robotP);
    config().blackboard->set<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP);
    LOG_INFO("Using robot pose: " << robotP.pose.position.x << ", " << robotP.pose.position.y);
    std::vector<FrontierPtr> frontier_list;
    LOG_WARN("Current search radius: " << frontierSearchPtr_->frontier_search_distance_);
    auto searchResult = frontierSearchPtr_->searchFrom(robotP.pose.position, frontier_list);
    if (searchResult != FrontierSearchResult::SUCCESSFUL_SEARCH)
    {
      LOG_INFO("No frontiers in current search radius.")
      explore_costmap_ros_->getCostmap()->getMutex()->unlock();
      config().blackboard->set<ExplorationErrorCode>(
        "error_code_id",
        ExplorationErrorCode::NO_FRONTIERS_IN_CURRENT_RADIUS);
      return BT::NodeStatus::FAILURE;
    }
    auto every_frontier = frontierSearchPtr_->getAllFrontiers();
    if (frontier_list.size() == 0) {
      LOG_INFO("No frontiers found in search. Incrementing search radius and returning BT Failure.");
      // EventLoggerInstance.endEvent("SearchForFrontiers", 0);
      explore_costmap_ros_->getCostmap()->getMutex()->unlock();
      LOG_INFO("Search was true but no frontiers found.");
      config().blackboard->set<ExplorationErrorCode>(
        "error_code_id",
        ExplorationErrorCode::NO_FRONTIERS_IN_CURRENT_RADIUS);
      return BT::NodeStatus::FAILURE;
    }
    LOG_INFO("Recieved " << frontier_list.size() << " frontiers");
    setOutput("frontier_list", frontier_list);
    setOutput("every_frontier", every_frontier);
    LOG_DEBUG("Set frontier outputs");
    rosVisualizerInstance.visualizeFrontier(
      frontier_list, every_frontier,
      explore_costmap_ros_->getLayeredCostmap()->getGlobalFrameID());
    LOG_DEBUG("Frontiers visualized");
    EventLoggerInstance.endEvent("SearchForFrontiers", 0);
    explore_costmap_ros_->getCostmap()->getMutex()->unlock();
    return BT::NodeStatus::SUCCESS;
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
  std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
};

class CleanupRoadMapBT : public BT::SyncActionNode
{
public:
  CleanupRoadMapBT(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<FullPathOptimizer> full_path_optimizer)
  : BT::SyncActionNode(name, config)
  {
    full_path_optimizer_ = full_path_optimizer;
    explore_costmap_ros_ = explore_costmap_ros;
    LOG_INFO("CleanupRoadMapBT Constructor");
  }

  BT::NodeStatus tick() override
  {
    LOG_FLOW("MODULE CleanupRoadMapBT");
    // LOG_INFO("Time since last clearance: " << EventLoggerInstance.getTimeSinceStart("clearRoadmap"));
    double time_between_cleanup;
    getInput("time_between_cleanup", time_between_cleanup);
    if (EventLoggerInstance.getTimeSinceStart("clearRoadmap") < time_between_cleanup) {
      return BT::NodeStatus::SUCCESS;
    }
    EventLoggerInstance.startEvent("clearRoadmap");
    EventLoggerInstance.startEvent("CleanupRoadMapBT");
    EventLoggerInstance.startEvent("roadmapReconstructionFull");
    bool correct_loop_closure_;
    getInput("correct_loop_closure", correct_loop_closure_);
    LOG_WARN("Reconstructing roadmap and clearing plan cache!");
    frontierRoadmapInstance.reConstructGraph(false, correct_loop_closure_);
    EventLoggerInstance.endEvent("roadmapReconstructionFull", 1);
    full_path_optimizer_->clearPlanCache();
    // TODO: make sure to add a thing such that the entire roadmap within a certain distance (max frontier search distance) is reconstructed periodically
    EventLoggerInstance.endEvent("CleanupRoadMapBT", 0);
    // frontierRoadmapInstance.countTotalItemsInSpatialMap();
    return BT::NodeStatus::SUCCESS;
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

class UpdateRoadmapBT : public BT::SyncActionNode
{
public:
  UpdateRoadmapBT(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
  : BT::SyncActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    LOG_INFO("UpdateRoadmapBT Constructor");
  }

  BT::NodeStatus tick() override
  {
    EventLoggerInstance.incrementPlanningCount();
    EventLoggerInstance.startEvent("UpdateRoadmapBT");
    LOG_FLOW("MODULE UpdateRoadmapBT");
    std::vector<FrontierPtr> frontier_list;
    getInput<std::vector<FrontierPtr>>("frontier_list", frontier_list);
    LOG_INFO("Recieved " << frontier_list.size() << " frontiers in UpdateRoadmapBT");
    geometry_msgs::msg::PoseStamped robotP;
    if (!config().blackboard->get<geometry_msgs::msg::PoseStamped>("latest_robot_pose", robotP)) {
      // Handle the case when "latest_robot_pose" is not found
      LOG_FATAL("Failed to retrieve latest_robot_pose from blackboard.");
      throw RoadmapExplorerException("Failed to retrieve latest_robot_pose from blackboard.");
    }
    frontierRoadmapInstance.addNodes(frontier_list, true);
    bool addPose;
    getInput("add_robot_pose_to_roadmap", addPose);
    if (addPose) {
      LOG_FLOW("Adding robot pose as frontier node.");
      frontierRoadmapInstance.addRobotPoseAsNode(robotP.pose, true);
    }
    EventLoggerInstance.startEvent("roadmapReconstruction");
    frontierRoadmapInstance.constructNewEdges(frontier_list);
    frontierRoadmapInstance.constructNewEdgeRobotPose(robotP.pose);
    // frontierRoadmapInstance.reConstructGraph();
    EventLoggerInstance.endEvent("roadmapReconstruction", 1);

    EventLoggerInstance.startEvent("publishRoadmap");
    frontierRoadmapInstance.publishRoadMap();
    EventLoggerInstance.endEvent("publishRoadmap", 2);
    // TODO: make sure to add a thing such that the entire roadmap within a certain distance (max frontier search distance) is reconstructed periodically
    EventLoggerInstance.endEvent("UpdateRoadmapBT", 0);
    // frontierRoadmapInstance.countTotalItemsInSpatialMap();
    // TODO: remove below line
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
      BT::InputPort<bool>("add_robot_pose_to_roadmap")};
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
};

class ProcessFrontierCostsBT : public BT::SyncActionNode
{
public:
  ProcessFrontierCostsBT(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<CostAssigner> cost_assigner_ptr,
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
  : BT::SyncActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    cost_assigner_ptr_ = cost_assigner_ptr;
    ros_node_ptr_ = ros_node_ptr;
    LOG_INFO("ProcessFrontierCostsBT Constructor");
  }

  BT::NodeStatus tick() override
  {
    EventLoggerInstance.startEvent("ProcessFrontierCosts");
    LOG_FLOW("MODULE ProcessFrontierCostsBT");
    auto frontierCostsRequestPtr = std::make_shared<GetFrontierCostsRequest>();
    auto frontierCostsResultPtr = std::make_shared<GetFrontierCostsResponse>();

    frontierCostsRequestPtr->prohibited_frontiers =
      *(config().blackboard->get<std::shared_ptr<std::vector<FrontierPtr>>>(
        "blacklisted_frontiers"));

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
      throw RoadmapExplorerException("Failed to retrieve latest_robot_pose from blackboard.");
    }
    if (!getInput<bool>(
        "force_grid_based_planning",
        frontierCostsRequestPtr->force_grid_base_planning))
    {
      BT::RuntimeError("No correct input recieved for force_grid_based_planning");
    }
    LOG_INFO("Request to get frontier costs sent");
    bool frontierCostsSuccess = cost_assigner_ptr_->getFrontierCosts(
      frontierCostsRequestPtr,
      frontierCostsResultPtr);
    if (frontierCostsSuccess == false) {
      LOG_INFO("Failed to receive response for getNextFrontier called from within the robot.");
      EventLoggerInstance.endEvent("ProcessFrontierCosts", 0);
      config().blackboard->set<ExplorationErrorCode>(
        "error_code_id",
        ExplorationErrorCode::COST_COMPUTATION_FAILURE);
      return BT::NodeStatus::FAILURE;
    }
    bool atleast_one_achievable_frontier = false;
    for (auto & fip : frontierCostsRequestPtr->frontier_list) {
      if (fip->isAchievable()) {
        atleast_one_achievable_frontier = true;
        break;
      }
    }
    if (!atleast_one_achievable_frontier) {
      LOG_INFO("No achievable frontiers left. Exiting.");
      config().blackboard->set<ExplorationErrorCode>(
        "error_code_id", ExplorationErrorCode::NO_ACHIEVABLE_FRONTIERS_LEFT);
      return BT::NodeStatus::FAILURE;
    }
    setOutput("frontier_costs_result", frontierCostsResultPtr->frontier_list);
    EventLoggerInstance.endEvent("ProcessFrontierCosts", 0);
    rosVisualizerInstance.visualizeFrontierMarker(
      frontierCostsResultPtr->frontier_list);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
      BT::InputPort<std::vector<std::vector<double>>>("every_frontier"),
      BT::InputPort<bool>("force_grid_based_planning"),
      BT::OutputPort<std::vector<FrontierPtr>>("frontier_costs_result")};
  }

  std::shared_ptr<FrontierSearch> frontierSearchPtr_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  std::shared_ptr<CostAssigner> cost_assigner_ptr_;
  std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
};

class OptimizeFullPath : public BT::SyncActionNode
{
public:
  OptimizeFullPath(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<FullPathOptimizer> full_path_optimizer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<nav2_util::LifecycleNode> node)
  : BT::SyncActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    full_path_optimizer_ = full_path_optimizer;
    node_ = node;
    LOG_INFO("OptimizeFullPath Constructor");
  }

  BT::NodeStatus tick() override
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
      throw RoadmapExplorerException("Failed to retrieve latest_robot_pose from blackboard.");
    }
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    auto return_state = full_path_optimizer_->getNextGoal(
      globalFrontierList, allocatedFrontier, robotP);
    if (return_state) {
      setOutput<FrontierPtr>("allocated_frontier", allocatedFrontier);
    } else {
      LOG_INFO("Full path optimization failed.");
      EventLoggerInstance.endEvent("OptimizeFullPath", 0);
      config().blackboard->set<ExplorationErrorCode>(
        "error_code_id", ExplorationErrorCode::FULL_PATH_OPTIMIZATION_FAILURE);
      return BT::NodeStatus::FAILURE;
    }
    EventLoggerInstance.endEvent("OptimizeFullPath", 0);
    setOutput<FrontierPtr>("allocated_frontier", allocatedFrontier);

    nav_msgs::msg::Path path;
    if (!full_path_optimizer_->refineAndPublishPath(robotP, allocatedFrontier, path)) {
      LOG_ERROR(
        "Failed to refine and publish path between robotP: " << robotP.pose.position.x << ", " << robotP.pose.position.y << " and " <<
          allocatedFrontier);
      config().blackboard->set<ExplorationErrorCode>(
        "error_code_id", ExplorationErrorCode::REFINED_PATH_COMPUTATION_FAILURE);
      blacklistFrontier(allocatedFrontier, config().blackboard);
      return BT::NodeStatus::FAILURE;
    }
    setOutput<nav_msgs::msg::Path>("optimized_path", path);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<FrontierPtr>>("frontier_costs_result"),
      BT::OutputPort<FrontierPtr>("allocated_frontier"),
      BT::OutputPort<nav_msgs::msg::Path>("optimized_path")};
  }

  std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  std::shared_ptr<nav2_util::LifecycleNode> node_;
};

class SendNav2Goal : public BT::StatefulActionNode
{
public:
  SendNav2Goal(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface,
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
  : BT::StatefulActionNode(name, config)
  {
    nav2_interface_ = nav2_interface;
    ros_node_ptr_ = ros_node_ptr;
    LOG_INFO("SendNav2Goal Constructor");
  }

  BT::NodeStatus onStart() override
  {
    LOG_FLOW("SendNav2Goal onStart");
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    getInput("allocated_frontier", allocatedFrontier);
    geometry_msgs::msg::PoseStamped goalPose;
    goalPose.header.frame_id = "map";
    goalPose.pose.position = allocatedFrontier->getGoalPoint();
    goalPose.pose.orientation = allocatedFrontier->getGoalOrientation();
    if (!nav2_interface_->canSendNewGoal()) {
      LOG_ERROR(
        "Nav2Interface cannot send new goal, goal is already active. Status:" <<
          nav2_interface_->getGoalStatus());
      // dont return failure if it's ongoing. It's probably on going from the previous iteration.
      if (nav2_interface_->getGoalStatus() != NavGoalStatus::ONGOING) {
        config().blackboard->set<ExplorationErrorCode>(
          "error_code_id", ExplorationErrorCode::UNHANDLED_ERROR);
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      nav2_interface_->sendGoal(goalPose);
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning()
  {
    LOG_DEBUG("SendNav2Goal onRunning");
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    getInput("allocated_frontier", allocatedFrontier);
    LOG_DEBUG("Sending goal " << allocatedFrontier);
    if (nav2_interface_->getGoalStatus() == NavGoalStatus::SENDING_GOAL) {
      LOG_INFO("Nav2 goal is being sent, waiting for response...");
      return BT::NodeStatus::RUNNING;
    }
    if (nav2_interface_->getGoalStatus() == NavGoalStatus::ONGOING) {
      LOG_INFO("Nav2 goal is ongoing, waiting for completion...");
      geometry_msgs::msg::PoseStamped goalPose;
      goalPose.header.frame_id = "map";
      goalPose.pose.position = allocatedFrontier->getGoalPoint();
      goalPose.pose.orientation = allocatedFrontier->getGoalOrientation();
      LOG_TRACE(
        "Current goal pose: " << goalPose.pose.position.x << ", " << goalPose.pose.position.y << ", " << goalPose.pose.orientation.z << ", " << goalPose.pose.orientation.w << ", " << goalPose.pose.orientation.x << ", " <<
          goalPose.pose.orientation.y);
      nav2_interface_->sendUpdatedGoal(goalPose);
      return BT::NodeStatus::RUNNING;
    }
    if (nav2_interface_->getGoalStatus() == NavGoalStatus::FAILED) {
      LOG_ERROR("Nav2 goal has aborted!");
      config().blackboard->set<ExplorationErrorCode>(
        "error_code_id", ExplorationErrorCode::NAV2_GOAL_ABORT);
      config().blackboard->set<FrontierPtr>(
        "latest_failed_frontier", allocatedFrontier);
      return BT::NodeStatus::FAILURE;
    }
    if (nav2_interface_->getGoalStatus() == NavGoalStatus::SUCCEEDED) {
      LOG_WARN("Nav2 goal has succeeded!");
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    LOG_WARN("SendNav2Goal onHalted");
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<FrontierPtr>("allocated_frontier")};
  }

  std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface_;
  std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
};

class CancelAllNav2Goals : public BT::SyncActionNode
{
public:
  CancelAllNav2Goals(
    const std::string & name, const BT::NodeConfiguration & config, std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface)
  : BT::SyncActionNode(name, config)
  {
    nav2_interface_ = nav2_interface;
    LOG_INFO("CancelAllNav2Goals Constructor");
  }

  BT::NodeStatus tick() override
  {
    EventLoggerInstance.startEvent("CancelAllNav2Goals");
    LOG_FLOW("MODULE CancelAllNav2Goals");
    nav2_interface_->cancelAllGoals();
    while (!nav2_interface_->isGoalTerminated() && rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      LOG_INFO("Waiting for Nav2 goal to be cancelled...");
    }
    LOG_WARN("Goal is terminated");
    return BT::NodeStatus::SUCCESS;
  }

  std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface_;
};

class BlacklistGoal : public BT::SyncActionNode
{
public:
  BlacklistGoal(
    const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    LOG_INFO("BlacklistGoal Constructor");
  }

  BT::NodeStatus tick() override
  {
    EventLoggerInstance.startEvent("BlacklistGoal");
    LOG_FLOW("MODULE BlacklistGoal");
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    if(!config().blackboard->get<FrontierPtr>("latest_failed_frontier", allocatedFrontier))
    {
      throw RoadmapExplorerException("Could not get latest allocated frontier from blackboard");
    }
    LOG_WARN("Setting blacklist " << allocatedFrontier);
    blacklistFrontier(allocatedFrontier, config().blackboard);
    return BT::NodeStatus::SUCCESS;
  }
};

}

namespace roadmap_explorer
{
RoadmapExplorationBT::RoadmapExplorationBT(std::shared_ptr<nav2_util::LifecycleNode> node, bool localisation_only_mode)
: bt_node_(node)
{
  LOG_INFO("Creating exploration costmap instance");
#ifdef ROS_DISTRO_HUMBLE
  explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "roadmap_explorer_costmap", "", "roadmap_explorer_costmap");
#elif ROS_DISTRO_JAZZY
  explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "roadmap_explorer_costmap", "", "roadmap_explorer_costmap", node->get_parameter("use_sim_time").as_bool());
#else
   #error Unsupported ROS DISTRO
#endif

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(bt_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Launch a thread to run the costmap node
  explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
  LOG_INFO("Created exploration costmap instance");

  explore_costmap_ros_->configure();
  explore_costmap_ros_->activate();

  blackboard = BT::Blackboard::create();
  blackboard->set<ExplorationErrorCode>("error_code_id", ExplorationErrorCode::NO_ERROR);

  EventLogger::createInstance();
  ParameterHandler::createInstance();
  parameterInstance.makeParameters(true, node);
  RosVisualizer::createInstance(bt_node_, explore_costmap_ros_->getCostmap());
  FrontierRoadMap::createInstance(explore_costmap_ros_, node);


  nav2_interface_ = std::make_shared<Nav2Interface<nav2_msgs::action::NavigateToPose>>(
    bt_node_,
    "navigate_to_pose",
    "goal_update");

  cost_assigner_ptr_ = std::make_shared<CostAssigner>(explore_costmap_ros_);
  frontierSearchPtr_ =
    std::make_shared<FrontierSearch>(*(explore_costmap_ros_->getLayeredCostmap()->getCostmap()));
  full_path_optimizer_ = std::make_shared<FullPathOptimizer>(bt_node_, explore_costmap_ros_);

  if (localisation_only_mode) {
    sensor_simulator_ = std::make_shared<SensorSimulator>(bt_node_, explore_costmap_ros_);
  }

  LOG_INFO("RoadmapExplorationBT::RoadmapExplorationBT()");
}

bool RoadmapExplorationBT::incrementFrontierSearchDistance()
{
  double increment_value = parameterInstance.getValue<double>("explorationBT.increment_search_distance_by");
  LOG_WARN("Incrementing frontier search distance by " << increment_value);
  auto result_distance = frontierSearchPtr_->incrementSearchDistance(increment_value);
  if (!result_distance) {
    LOG_ERROR("Maximum frontier search distance exceeded. Returning false.");
    return false;
  }
  return true;
}

bool RoadmapExplorationBT::resetFrontierSearchDistance()
{
  LOG_DEBUG("Resetting frontier search distance to default value");
  frontierSearchPtr_->resetSearchDistance();
  return true;
}

RoadmapExplorationBT::~RoadmapExplorationBT()
{
  LOG_INFO("RoadmapExplorationBT::~RoadmapExplorationBT()");
  FrontierRoadMap::destroyInstance();
  RosVisualizer::destroyInstance();
  ParameterHandler::destroyInstance();
  EventLogger::destroyInstance();
  explore_costmap_ros_->deactivate();
  explore_costmap_ros_->cleanup();
  explore_costmap_ros_.reset();
  explore_costmap_thread_.reset();
}

bool RoadmapExplorationBT::makeBTNodes()
{
  EventLoggerInstance.startEvent("clearRoadmap");
  EventLoggerInstance.startEvent("replanTimeout");

  pluginlib::ClassLoader<roadmap_explorer::BTPlugin> loader(
  "roadmap_explorer",           // your package name
  "roadmap_explorer::BTPlugin"  // the interface
  );

  LOG_WARN("Loading BT plugins (size): " << loader.getDeclaredClasses().size());

  for (auto & lookup_name : loader.getDeclaredClasses()) {
    auto plugin = loader.createSharedInstance(lookup_name);
    plugin->registerNodes(factory, bt_node_, explore_costmap_ros_, tf_buffer_);
    LOG_WARN("Loaded BT plugin: " << lookup_name.c_str());
  }

  // ------------------- Nodes ----------------------------------------------

  BT::NodeBuilder builder_update_boundary_polygon =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<UpdateBoundaryPolygonBT>(
        name, config, cost_assigner_ptr_, bt_node_);
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
      return std::make_unique<UpdateRoadmapBT>(name, config, explore_costmap_ros_);
    };
  factory.registerBuilder<UpdateRoadmapBT>("UpdateFrontierRoadmap", builder_update_roadmap_data);

  BT::NodeBuilder builder_cleanup_roadmap_data =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<CleanupRoadMapBT>(
        name, config, explore_costmap_ros_,
        full_path_optimizer_);
    };
  factory.registerBuilder<CleanupRoadMapBT>("CleanupRoadmap", builder_cleanup_roadmap_data);

  BT::NodeBuilder builder_frontier_costs =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<ProcessFrontierCostsBT>(
        name, config, explore_costmap_ros_, cost_assigner_ptr_, bt_node_);
    };
  factory.registerBuilder<ProcessFrontierCostsBT>("ProcessFrontierCosts", builder_frontier_costs);

  BT::NodeBuilder builder_full_path_optimizer =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<OptimizeFullPath>(
        name, config, full_path_optimizer_,
        explore_costmap_ros_, bt_node_);
    };
  factory.registerBuilder<OptimizeFullPath>("OptimizeFullPath", builder_full_path_optimizer);

  BT::NodeBuilder builder_send_goal =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<SendNav2Goal>(name, config, nav2_interface_, bt_node_);
    };
  factory.registerBuilder<SendNav2Goal>("SendNav2Goal", builder_send_goal);

  BT::NodeBuilder builder_cancel_all_goals =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<CancelAllNav2Goals>(name, config, nav2_interface_);
    };
  factory.registerBuilder<CancelAllNav2Goals>("CancelAllNav2Goals", builder_cancel_all_goals);

  BT::NodeBuilder builder_blacklist_goal =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<BlacklistGoal>(name, config);
    };
  factory.registerBuilder<BlacklistGoal>("BlacklistGoal", builder_blacklist_goal);

  // -------------------- Control and decorators -----------------------------
  factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
  factory.registerNodeType<nav2_behavior_tree::RateController>("RateController");
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");

  behaviour_tree =
    factory.createTreeFromFile(
    parameterInstance.getValue<std::string>(
      "explorationBT.bt_xml_path"), blackboard);

  blackboard->set<std::shared_ptr<std::vector<FrontierPtr>>>(
    "blacklisted_frontiers",
    std::make_shared<std::vector<FrontierPtr>>());
  
  {
    /* This entire thing is a workaround for now. These should be addressed once there is the capability to update the obstacle rejection thresholds in the Nav2 global planners*/
    auto client_planner = std::make_shared<rclcpp::AsyncParametersClient>(bt_node_, "/planner_server");
    auto client_global_costmap = std::make_shared<rclcpp::AsyncParametersClient>(bt_node_, "/global_costmap/global_costmap");
    
    if (!client_planner->wait_for_service(std::chrono::seconds(1)) || !client_global_costmap->wait_for_service(std::chrono::seconds(1)))
    {
      LOG_ERROR("Dynamic parameter service unavailable");
      return false;
    }
    
    auto future = client_global_costmap->set_parameters({ rclcpp::Parameter("robot_radius", 0.10) });
    future.wait();
    
    const auto results = future.get();
    for (const auto & result : results)
    {
      if (!result.successful)
      {
        LOG_ERROR("Failed to set parameter robot_radius on node global_costmap");
        return false;
      }
    }
    
    future = client_planner->set_parameters({ rclcpp::Parameter("GridBased.allow_unknown", true) });
    future.wait();
    const auto results2 = future.get();
    for (const auto & result : results2)
    {
      if (!result.successful)
      {
        LOG_ERROR("Failed to set parameter allow unknown on node planner_server");
        return false;
      }
    }
  }
  return true;

}

uint16_t RoadmapExplorationBT::tickOnceWithSleep()
{
  int bt_sleep_duration = parameterInstance.getValue<int64_t>("explorationBT.bt_sleep_ms");
  std::this_thread::sleep_for(std::chrono::milliseconds(bt_sleep_duration));
  if(sensor_simulator_)
  {
    sensor_simulator_->startMappingIfNotStarted();
  }
  explore_costmap_ros_->resume();
  
  if(!explore_costmap_ros_->isCurrent()) {
    LOG_WARN("Waiting for explore costmap to be current.");
    return ExploreActionResult::NO_ERROR;
  }

#ifdef ROS_DISTRO_HUMBLE
  auto status = behaviour_tree.tickRoot();
#elif ROS_DISTRO_JAZZY
  auto status = behaviour_tree.tickOnce();
#else
  #error Unsupported ROS DISTRO
#endif

  uint16_t return_value = ExploreActionResult::NO_ERROR;
  if (status == BT::NodeStatus::FAILURE) {
    if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::NO_FRONTIERS_IN_CURRENT_RADIUS)
    {
      LOG_ERROR(
        "Behavior Tree tick returned FAILURE due to no frontiers in current search radius.");
      incrementFrontierSearchDistance();
    } else if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::MAX_FRONTIER_SEARCH_RADIUS_EXCEEDED)
    {
      LOG_ERROR("Behavior Tree tick returned FAILURE due to max frontier search radius exceeded.");
    } else if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::COST_COMPUTATION_FAILURE)
    {
      LOG_ERROR("Behavior Tree tick returned FAILURE due to cost computation failure.");
    } else if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::NO_ACHIEVABLE_FRONTIERS_LEFT)
    {
      LOG_ERROR("Behavior Tree tick returned FAILURE due to no achievable frontiers left.");
      incrementFrontierSearchDistance();
    } else if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::FULL_PATH_OPTIMIZATION_FAILURE)
    {
      LOG_ERROR("Behavior Tree tick returned FAILURE due to full path optimization failure.");
    } else if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::REFINED_PATH_COMPUTATION_FAILURE)
    {
      LOG_ERROR("Behavior Tree tick returned FAILURE due to refined path computation failure.");
    } else if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::UNHANDLED_ERROR)
    {
      return_value = ExploreActionResult::UNKNOWN;
      LOG_ERROR("Behavior Tree tick returned FAILURE with unhandled error code.");
    } else if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::NAV2_GOAL_ABORT)
    {
      LOG_ERROR("Behavior Tree tick returned FAILURE due to Nav2 goal abort.");
      if (parameterInstance.getValue<bool>("explorationBT.abort_exploration_on_nav2_abort")) {
        LOG_ERROR("Aborting exploration due to Nav2 goal abort.");
        return_value = ExploreActionResult::NAV2_INTERNAL_FAULT;
      } else {
        LOG_WARN("Continuing exploration despite Nav2 goal abort. The frontier was blacklisted.");
      }
    } else if (blackboard->get<ExplorationErrorCode>("error_code_id") ==
      ExplorationErrorCode::NO_ERROR)
    {
      LOG_WARN("Failure without error. Continuing.") 
    } else {
      throw RoadmapExplorerException("Behavior Tree tick returned FAILURE with unknown error code.");
    }
  }
  else
  {
    resetFrontierSearchDistance();
  }
  LOG_DEBUG("TICKED ONCE");

  return return_value;
}

void RoadmapExplorationBT::halt()
{
  LOG_INFO("RoadmapExplorationBT::halt()");
  behaviour_tree.haltTree();
  if(sensor_simulator_)
  {
    sensor_simulator_->stopMapping();
  }
  explore_costmap_ros_->pause();
  nav2_interface_->cancelAllGoals();
  while (!nav2_interface_->isGoalTerminated() && rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    LOG_INFO("Waiting for Nav2 goal to be cancelled...");
  }
  LOG_WARN("Goal is terminated");
  // if(save_exploration_data)
  // {
  //   if (sensor_simulator_ == nullptr)
  //   {
  //     LOG_FATAL("Asked to save map but sensor simulator is inactive. Set to localisation only mode and turn on sensor simulator to save explored occupancy map and the corresponding roadmap!");
  //     return;
  //   }
  //   sensor_simulator_->saveMap("instance1", "/home/suchetan/.ros/roadmap-explorer");
  // }
}

}
