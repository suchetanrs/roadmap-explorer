#include <roadmap_explorer/ExplorationBT.hpp>
namespace roadmap_explorer
{

class WaitForCurrent : public BT::StatefulActionNode
{
public:
  WaitForCurrent(
    const std::string & name, const BT::NodeConfiguration & config,
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

class UpdateBoundaryPolygonBT : public BT::SyncActionNode
{
public:
  UpdateBoundaryPolygonBT(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<CostAssigner> cost_assigner_ptr, rclcpp::Node::SharedPtr ros_node_ptr)
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
    for (int i = 0; i < config_.size(); i += 2) {
      geometry_msgs::msg::Point32 point;
      point.x = config_[i];
      point.y = config_[i + 1];
      LOG_DEBUG("Adding point to boundary: " << point.x << ", " << point.y);
      explore_boundary_.polygon.points.push_back(point);
    }

    auto updateBoundaryResult = cost_assigner_ptr_->updateBoundaryPolygon(explore_boundary_);
    LOG_DEBUG("Adding update boundary polygon for spin.");
    if (updateBoundaryResult == true) {
      LOG_INFO("Region boundary set");
    } else {
      LOG_ERROR("Failed to receive response for updateBoundaryPolygon");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  std::shared_ptr<CostAssigner> cost_assigner_ptr_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
  std::vector<double> config_;
};

class SearchForFrontiersBT : public BT::SyncActionNode
{
public:
  SearchForFrontiersBT(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<FrontierSearch> frontierSearchPtr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    rclcpp::Node::SharedPtr ros_node_ptr)
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

class CleanupRoadMapBT : public BT::SyncActionNode
{
public:
  CleanupRoadMapBT(
    const std::string & name, const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr ros_node_ptr,
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
    rclcpp::Node::SharedPtr ros_node_ptr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
  : BT::SyncActionNode(name, config)
  {
    explore_costmap_ros_ = explore_costmap_ros;
    LOG_INFO("UpdateRoadmapBT Constructor");
  }

  BT::NodeStatus tick() override
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
    rclcpp::Node::SharedPtr ros_node_ptr)
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

    config().blackboard->get<std::vector<FrontierPtr>>(
      "blacklisted_frontiers", frontierCostsRequestPtr->prohibited_frontiers);

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
    bool frontierCostsSuccess = cost_assigner_ptr_->getFrontierCosts(
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

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<FrontierPtr>>("frontier_list"),
      BT::InputPort<std::vector<std::vector<double>>>("every_frontier"),
      BT::OutputPort<std::vector<FrontierPtr>>("frontier_costs_result")};
  }

  std::shared_ptr<FrontierSearch> frontierSearchPtr_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  std::shared_ptr<CostAssigner> cost_assigner_ptr_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
};

class OptimizeFullPath : public BT::SyncActionNode
{
public:
  OptimizeFullPath(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<FullPathOptimizer> full_path_optimizer,
    std::shared_ptr<CostAssigner> cost_assigner_ptr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    rclcpp::Node::SharedPtr node)
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
      throw std::runtime_error("Failed to retrieve latest_robot_pose from blackboard.");
    }
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    auto return_state = full_path_optimizer_->getNextGoal(
      globalFrontierList, allocatedFrontier, 3,
      robotP);
    if (return_state) {
      setOutput<FrontierPtr>("allocated_frontier", allocatedFrontier);
    } else {
      double increment_value = 0.1;
      getInput("increment_search_distance_by", increment_value);
      EventLoggerInstance.endEvent("OptimizeFullPath", 0);
      return BT::NodeStatus::FAILURE;
    }
    EventLoggerInstance.endEvent("OptimizeFullPath", 0);
    setOutput<FrontierPtr>("allocated_frontier", allocatedFrontier);

    nav_msgs::msg::Path path;
    if (!full_path_optimizer_->refineAndPublishPath(robotP, allocatedFrontier, path)) {
      LOG_ERROR(
        "Failed to refine and publish path between robotP: " << robotP.pose.position.x << ", " << robotP.pose.position.y << " and " <<
          allocatedFrontier);
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
  rclcpp::Node::SharedPtr node_;
};

class SendNav2Goal : public BT::StatefulActionNode
{
public:
  SendNav2Goal(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface,
    rclcpp::Node::SharedPtr ros_node_ptr)
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
    if(!nav2_interface_->canSendNewGoal())
    {
      LOG_ERROR("Nav2Interface cannot send new goal, goal is already active. Status:" << nav2_interface_->getGoalStatus());
      return BT::NodeStatus::FAILURE;
    }
    nav2_interface_->sendGoal(goalPose);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning()
  {
    LOG_DEBUG("SendNav2Goal onRunning");
    FrontierPtr allocatedFrontier = std::make_shared<Frontier>();
    getInput("allocated_frontier", allocatedFrontier);
    if(nav2_interface_->getGoalStatus() == NavGoalStatus::SENDING_GOAL) {
      LOG_INFO("Nav2 goal is being sent, waiting for response...");
      return BT::NodeStatus::RUNNING;
    }
    if (nav2_interface_->getGoalStatus() == NavGoalStatus::ONGOING) {
      LOG_INFO("Nav2 goal is ongoing, waiting for completion...");
      geometry_msgs::msg::PoseStamped goalPose;
      goalPose.header.frame_id = "map";
      goalPose.pose.position = allocatedFrontier->getGoalPoint();
      goalPose.pose.orientation = allocatedFrontier->getGoalOrientation();
      LOG_TRACE("Current goal pose: " << goalPose.pose.position.x << ", " << goalPose.pose.position.y << ", " << goalPose.pose.orientation.z << ", " << goalPose.pose.orientation.w << ", " << goalPose.pose.orientation.x << ", " << goalPose.pose.orientation.y);
      nav2_interface_->sendUpdatedGoal(goalPose);
      return BT::NodeStatus::RUNNING;
    }
    if(nav2_interface_->getGoalStatus() == NavGoalStatus::FAILED) {
      LOG_ERROR("Nav2 goal has aborted!");
      blacklisted_frontiers_.push_back(allocatedFrontier);
      config().blackboard->set<std::vector<FrontierPtr>>("blacklisted_frontiers", blacklisted_frontiers_);
      return BT::NodeStatus::FAILURE;
    }
    if(nav2_interface_->getGoalStatus() == NavGoalStatus::SUCCEEDED) {
      LOG_WARN("Nav2 goal has succeeded!");
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted()
  {
    LOG_WARN("SendNav2Goal onHalted");
    nav2_interface_->cancelAllGoals();
    while(!nav2_interface_->isGoalTerminated())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      LOG_INFO("Waiting for Nav2 goal to be cancelled...");
    }
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<FrontierPtr>("allocated_frontier")};
  }

  std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface_;
  rclcpp::Node::SharedPtr ros_node_ptr_;
  std::vector<FrontierPtr> blacklisted_frontiers_;
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
  parameterInstance.makeParameters(true, node);

  LOG_TRACE("Declared BT params");
  nav2_interface_ = std::make_shared<Nav2Interface<nav2_msgs::action::NavigateToPose>>(bt_node_, "navigate_to_pose", "goal_update");

  explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "roadmap_explorer_costmap", "", "roadmap_explorer_costmap");
  explore_costmap_ros_->configure();
  // Launch a thread to run the costmap node
  explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
  explore_costmap_ros_->activate();
  LOG_TRACE("Created costmap instance");

  RosVisualizer::createInstance(bt_node_, explore_costmap_ros_->getCostmap());
  FrontierRoadMap::createInstance(explore_costmap_ros_, node);
  LOG_TRACE("Created ros visualizer instance");
  cost_assigner_ptr_ = std::make_shared<CostAssigner>(explore_costmap_ros_);

  frontierSearchPtr_ = std::make_shared<FrontierSearch>(*(explore_costmap_ros_->getLayeredCostmap()->getCostmap()));
  full_path_optimizer_ = std::make_shared<FullPathOptimizer>(bt_node_, explore_costmap_ros_);

  LOG_INFO("FrontierExplorationServer::FrontierExplorationServer()");
}

FrontierExplorationServer::~FrontierExplorationServer()
{
  LOG_INFO("FrontierExplorationServer::~FrontierExplorationServer()");
  explore_costmap_ros_.reset();
  explore_costmap_thread_.reset();
  RosVisualizer::getInstance().cleanupInstance();
  FrontierRoadMap::getInstance().cleanupInstance();
  parameterInstance.cleanupInstance();
}

void FrontierExplorationServer::makeBTNodes()
{
  while (!explore_costmap_ros_->isCurrent()) {
    LOG_WARN("Waiting for explore costmap to be current.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  EventLoggerInstance.startEvent("clearRoadmap");
  EventLoggerInstance.startEvent("replanTimeout");

  BT::NodeBuilder builder_wait_for_current =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<WaitForCurrent>(name, config, explore_costmap_ros_, bt_node_);
    };
  factory.registerBuilder<WaitForCurrent>("WaitForCurrent", builder_wait_for_current);

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
        name, config, explore_costmap_ros_, cost_assigner_ptr_, bt_node_);
    };
  factory.registerBuilder<ProcessFrontierCostsBT>("ProcessFrontierCosts", builder_frontier_costs);

  BT::NodeBuilder builder_full_path_optimizer =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<OptimizeFullPath>(
        name, config, full_path_optimizer_, cost_assigner_ptr_,
        explore_costmap_ros_, bt_node_);
    };
  factory.registerBuilder<OptimizeFullPath>("OptimizeFullPath", builder_full_path_optimizer);

  BT::NodeBuilder builder_send_goal =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<SendNav2Goal>(name, config, nav2_interface_, bt_node_);
    };
  factory.registerBuilder<SendNav2Goal>("SendNav2Goal", builder_send_goal);

  factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
  factory.registerNodeType<nav2_behavior_tree::RateController>("RateController");

  behaviour_tree = factory.createTreeFromFile(parameterInstance.getValue<std::string>("explorationBT.bt_xml_path"), blackboard);
}

void FrontierExplorationServer::run()
{
  while (rclcpp::ok()) {
    if (exploration_active_) {
      int bt_sleep_duration = parameterInstance.getValue<int64_t>("explorationBT.bt_sleep_ms");
      behaviour_tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(bt_sleep_duration));
      LOG_DEBUG("TICKED ONCE");
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}
