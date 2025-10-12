#include <roadmap_explorer/ExplorationBT.hpp>

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
#elif ROS_DISTRO_KILTED
  explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "roadmap_explorer_costmap", "", node->get_parameter("use_sim_time").as_bool());
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
  parameterInstance.makeParameters(node);
  RosVisualizer::createInstance(bt_node_, explore_costmap_ros_->getCostmap());
  FrontierRoadMap::createInstance(explore_costmap_ros_, node);


  nav2_interface_ = std::make_shared<Nav2Interface<nav2_msgs::action::NavigateToPose>>(
    bt_node_,
    "navigate_to_pose",
    "goal_update");

  cost_assigner_ptr_ = std::make_shared<CostAssigner>(explore_costmap_ros_, bt_node_);
  

  // initialize frontier search plugin

  nav2_util::declare_parameter_if_not_declared(
      bt_node_, "frontierSearch.plugin_name", rclcpp::ParameterValue(
      "GridBasedBFS"));
  auto plugin_name = bt_node_->get_parameter("frontierSearch.plugin_name").as_string();
  nav2_util::declare_parameter_if_not_declared(
      bt_node_, "frontierSearch." + plugin_name + ".plugin_type", rclcpp::ParameterValue(
      "roadmap_explorer::FrontierBFSearch"));
  auto plugin_type = bt_node_->get_parameter("frontierSearch." + plugin_name + ".plugin_type").as_string();
  // Try to load frontier search plugin, fallback to direct instantiation
  try {
    frontier_search_loader_ = std::make_shared<pluginlib::ClassLoader<FrontierSearchBase>>(
      "roadmap_explorer", "roadmap_explorer::FrontierSearchBase");
    frontierSearchPtr_ = frontier_search_loader_->createSharedInstance(plugin_type);
    frontierSearchPtr_->configure(explore_costmap_ros_, "frontierSearch." + plugin_name, bt_node_);
    LOG_INFO("Loaded frontier search plugin: " << plugin_type);
  } catch (const std::exception& e) {
    LOG_WARN("Failed to load frontier search plugin, using direct instantiation: " << e.what());
    throw std::runtime_error("Failed to load frontier search plugin");
  }
  resetFrontierSearchDistance();
  
  full_path_optimizer_ = std::make_shared<FullPathOptimizer>(bt_node_, explore_costmap_ros_);

  if (localisation_only_mode) {
    sensor_simulator_ = std::make_shared<SensorSimulator>(bt_node_, explore_costmap_ros_);
  }

  LOG_INFO("RoadmapExplorationBT::RoadmapExplorationBT()");
}

bool RoadmapExplorationBT::incrementFrontierSearchDistance()
{
  auto current_frontier_search_distance = blackboard->get<double>("current_frontier_search_distance");
  double increment_value = parameterInstance.getValue<double>("frontierSearch.increment_search_distance_by");
  LOG_WARN("Incrementing frontier search distance by " << increment_value);
  auto distance_to_set = current_frontier_search_distance + increment_value;
  if (distance_to_set > parameterInstance.getValue<double>("frontierSearch.max_permissable_frontier_search_distance")) {
    return false;
  }
  blackboard->set<double>("current_frontier_search_distance", distance_to_set);
  return true;
}

bool RoadmapExplorationBT::resetFrontierSearchDistance()
{
  LOG_DEBUG("Resetting frontier search distance to default value");
  auto original_search_distance = parameterInstance.getValue<double>(
    "frontierSearch.frontier_search_distance");
  blackboard->set<double>("current_frontier_search_distance", original_search_distance);
  return true;
}

RoadmapExplorationBT::~RoadmapExplorationBT()
{
  LOG_INFO("RoadmapExplorationBT::~RoadmapExplorationBT()");
  
  // Reset plugin instances before destroying class loaders
  frontierSearchPtr_.reset();
  cost_assigner_ptr_.reset();
  full_path_optimizer_.reset();
  sensor_simulator_.reset();
  nav2_interface_.reset();
  
  FrontierRoadMap::destroyInstance();
  RosVisualizer::destroyInstance();
  ParameterHandler::destroyInstance();
  EventLogger::destroyInstance();
  explore_costmap_ros_->deactivate();
  explore_costmap_ros_->cleanup();
  explore_costmap_ros_.reset();
  explore_costmap_thread_.reset();
  
  // Now it's safe to destroy the class loaders
  frontier_search_loader_.reset();
  bt_plugin_loader_.reset();
}

bool RoadmapExplorationBT::makeBTNodes()
{
  EventLoggerInstance.startEvent("clearRoadmap");
  EventLoggerInstance.startEvent("replanTimeout");

  // Create BTContext with all shared resources
  auto bt_context = std::make_shared<BTContext>();
  bt_context->node = bt_node_;
  bt_context->explore_costmap_ros = explore_costmap_ros_;
  bt_context->tf_buffer = tf_buffer_;
  bt_context->cost_assigner = cost_assigner_ptr_;
  bt_context->frontier_search = frontierSearchPtr_;
  bt_context->full_path_optimizer = full_path_optimizer_;
  bt_context->nav2_interface = nav2_interface_;

  bt_plugin_loader_ = std::make_shared<pluginlib::ClassLoader<roadmap_explorer::BTPlugin>>(
  "roadmap_explorer",           // your package name
  "roadmap_explorer::BTPlugin"  // the interface
  );

  LOG_WARN("Loading BT plugins (size): " << bt_plugin_loader_->getDeclaredClasses().size());

  for (auto & lookup_name : bt_plugin_loader_->getDeclaredClasses()) {
    auto plugin = bt_plugin_loader_->createSharedInstance(lookup_name);
    plugin->registerNodes(factory, bt_context);
    LOG_WARN("Loaded BT plugin: " << lookup_name.c_str());
  }
  
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
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
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
