#include <roadmap_explorer/Parameters.hpp>

std::unique_ptr<ParameterHandler> ParameterHandler::parameterHandlerPtr_ = nullptr;
std::recursive_mutex ParameterHandler::instanceMutex_;

ParameterHandler::ParameterHandler()
{
  LOG_INFO("ParameterHandler::ParameterHandler");
}

ParameterHandler::~ParameterHandler()
{
  LOG_INFO("ParameterHandler::~ParameterHandler");
  dynamic_param_callback_handle_.reset();
  parameter_map_.clear();
}

void ParameterHandler::makeParameters(
  bool use_ros_parameters,
  std::shared_ptr<nav2_util::LifecycleNode> node)
{
  if (use_ros_parameters) {
    makeParametersROS(node);
  } else {
    makeParametersYAMLcpp();
  }
}

void ParameterHandler::makeParametersROS(std::shared_ptr<nav2_util::LifecycleNode> node)
{
  // --- frontierSearch ---
  nav2_util::declare_parameter_if_not_declared(
    node, "frontierSearch.min_frontier_cluster_size", rclcpp::ParameterValue(
      1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "frontierSearch.max_frontier_cluster_size", rclcpp::ParameterValue(
      20.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "frontierSearch.frontier_search_distance", rclcpp::ParameterValue(
      50.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "frontierSearch.lethal_threshold", rclcpp::ParameterValue(
      160));
  nav2_util::declare_parameter_if_not_declared(
    node, "frontierSearch.max_frontier_search_distance", rclcpp::ParameterValue(
      100.0));

  parameter_map_["frontierSearch.min_frontier_cluster_size"] = node->get_parameter(
    "frontierSearch.min_frontier_cluster_size").as_double();
  parameter_map_["frontierSearch.max_frontier_cluster_size"] = node->get_parameter(
    "frontierSearch.max_frontier_cluster_size").as_double();
  parameter_map_["frontierSearch.frontier_search_distance"] = node->get_parameter(
    "frontierSearch.frontier_search_distance").as_double();
  parameter_map_["frontierSearch.lethal_threshold"] = node->get_parameter(
    "frontierSearch.lethal_threshold").as_int();
  parameter_map_["frontierSearch.max_frontier_search_distance"] = node->get_parameter(
    "frontierSearch.max_frontier_search_distance").as_double();

  // --- costCalculator ---
  nav2_util::declare_parameter_if_not_declared(
    node, "costCalculator.max_camera_depth", rclcpp::ParameterValue(
      2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "costCalculator.delta_theta", rclcpp::ParameterValue(
      0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, "costCalculator.camera_fov", rclcpp::ParameterValue(
      1.04));
  nav2_util::declare_parameter_if_not_declared(
    node, "costCalculator.factor_of_max_is_min", rclcpp::ParameterValue(
      0.70));
  nav2_util::declare_parameter_if_not_declared(
    node, "costCalculator.closeness_rejection_threshold", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "costCalculator.planner_allow_unknown", rclcpp::ParameterValue(
      true));
  nav2_util::declare_parameter_if_not_declared(
    node, "costCalculator.max_planning_distance_roadmap", rclcpp::ParameterValue(
      6.0));

  parameter_map_["costCalculator.max_camera_depth"] = node->get_parameter(
    "costCalculator.max_camera_depth").as_double();
  parameter_map_["costCalculator.delta_theta"] =
    node->get_parameter("costCalculator.delta_theta").as_double();
  parameter_map_["costCalculator.camera_fov"] =
    node->get_parameter("costCalculator.camera_fov").as_double();
  parameter_map_["costCalculator.factor_of_max_is_min"] = node->get_parameter(
    "costCalculator.factor_of_max_is_min").as_double();
  parameter_map_["costCalculator.closeness_rejection_threshold"] = node->get_parameter(
    "costCalculator.closeness_rejection_threshold").as_double();
  parameter_map_["costCalculator.planner_allow_unknown"] = node->get_parameter(
    "costCalculator.planner_allow_unknown").as_bool();
  parameter_map_["costCalculator.max_planning_distance_roadmap"] = node->get_parameter(
    "costCalculator.max_planning_distance_roadmap").as_double();

  // --- costAssigner ---
  std::vector<std::string> default_cost_calculation_methods =
  {"RoadmapPlannerDistance", "ArrivalInformation"};
  // {"A*PlannerDistance", "ArrivalInformation"};
  // {"EuclideanDistance", "ArrivalInformation"};
  // {"RandomCosts"};
  // {};
  nav2_util::declare_parameter_if_not_declared(
    node, "costAssigner.cost_calculation_methods", rclcpp::ParameterValue(
      default_cost_calculation_methods));

  parameter_map_["costAssigner.cost_calculation_methods"] =
    node->get_parameter("costAssigner.cost_calculation_methods").as_string_array();

  // --- frontierRoadmap ---
  nav2_util::declare_parameter_if_not_declared(
    node,
    "frontierRoadmap.max_graph_reconstruction_distance", rclcpp::ParameterValue(
      25.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "frontierRoadmap.grid_cell_size", rclcpp::ParameterValue(
      1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "frontierRoadmap.radius_to_decide_edges", rclcpp::ParameterValue(
      6.1));
  nav2_util::declare_parameter_if_not_declared(
    node,
    "frontierRoadmap.min_distance_between_two_frontier_nodes", rclcpp::ParameterValue(
      0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    "frontierRoadmap.min_distance_between_robot_pose_and_node", rclcpp::ParameterValue(
      0.25));

  parameter_map_["frontierRoadmap.max_graph_reconstruction_distance"] = node->get_parameter(
    "frontierRoadmap.max_graph_reconstruction_distance").as_double();
  parameter_map_["frontierRoadmap.grid_cell_size"] = node->get_parameter(
    "frontierRoadmap.grid_cell_size").as_double();
  parameter_map_["frontierRoadmap.radius_to_decide_edges"] = node->get_parameter(
    "frontierRoadmap.radius_to_decide_edges").as_double();
  parameter_map_["frontierRoadmap.min_distance_between_two_frontier_nodes"] = node->get_parameter(
    "frontierRoadmap.min_distance_between_two_frontier_nodes").as_double();
  parameter_map_["frontierRoadmap.min_distance_between_robot_pose_and_node"] = node->get_parameter(
    "frontierRoadmap.min_distance_between_robot_pose_and_node").as_double();

  // --- fullPathOptimizer ---
  nav2_util::declare_parameter_if_not_declared(
    node,
    "fullPathOptimizer.num_frontiers_in_local_area", rclcpp::ParameterValue(
      5.0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    "fullPathOptimizer.local_frontier_search_radius", rclcpp::ParameterValue(
      12.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "fullPathOptimizer.add_yaw_to_tsp", rclcpp::ParameterValue(
      false));
  nav2_util::declare_parameter_if_not_declared(
    node,
    "fullPathOptimizer.add_distance_to_robot_to_tsp", rclcpp::ParameterValue(
      false));

  parameter_map_["fullPathOptimizer.num_frontiers_in_local_area"] = node->get_parameter(
    "fullPathOptimizer.num_frontiers_in_local_area").as_double();
  parameter_map_["fullPathOptimizer.local_frontier_search_radius"] = node->get_parameter(
    "fullPathOptimizer.local_frontier_search_radius").as_double();
  parameter_map_["fullPathOptimizer.add_yaw_to_tsp"] = node->get_parameter(
    "fullPathOptimizer.add_yaw_to_tsp").as_bool();
  parameter_map_["fullPathOptimizer.add_distance_to_robot_to_tsp"] = node->get_parameter(
    "fullPathOptimizer.add_distance_to_robot_to_tsp").as_bool();

  // --- goalHysteresis ---
  nav2_util::declare_parameter_if_not_declared(
    node, "goalHysteresis.use_euclidean_distance", rclcpp::ParameterValue(
      false));
  nav2_util::declare_parameter_if_not_declared(
    node, "goalHysteresis.use_roadmap_planner_distance", rclcpp::ParameterValue(
      true));

  parameter_map_["goalHysteresis.use_euclidean_distance"] = node->get_parameter(
    "goalHysteresis.use_euclidean_distance").as_bool();
  parameter_map_["goalHysteresis.use_roadmap_planner_distance"] = node->get_parameter(
    "goalHysteresis.use_roadmap_planner_distance").as_bool();

  // --- explorationBT ---
  nav2_util::declare_parameter_if_not_declared(
    node, "explorationBT.bt_sleep_ms", rclcpp::ParameterValue(
      70));
  nav2_util::declare_parameter_if_not_declared(
    node, "explorationBT.nav2_bt_xml", rclcpp::ParameterValue(
      roadmap_explorer_dir + "/xml/explore_to_pose.xml"));
  nav2_util::declare_parameter_if_not_declared(
    node, "explorationBT.bt_xml_path", rclcpp::ParameterValue(
      roadmap_explorer_dir + "/xml/exploration.xml"));
  std::vector default_exploration_boundary =
  {310.0, 260.0, 310.0, -120.0, -70.0, -120.0, -70.0, 260.0};
  nav2_util::declare_parameter_if_not_declared(
    node, "explorationBT.exploration_boundary", rclcpp::ParameterValue(
      default_exploration_boundary));
  nav2_util::declare_parameter_if_not_declared(
    node,
    "explorationBT.abort_exploration_on_nav2_abort", rclcpp::ParameterValue(
      true));
  nav2_util::declare_parameter_if_not_declared(
    node, "explorationBT.increment_search_distance_by", rclcpp::ParameterValue(
      0.1));

  parameter_map_["explorationBT.bt_sleep_ms"] =
    node->get_parameter("explorationBT.bt_sleep_ms").as_int();
  parameter_map_["explorationBT.nav2_bt_xml"] =
    node->get_parameter("explorationBT.nav2_bt_xml").as_string();
  parameter_map_["explorationBT.bt_xml_path"] =
    node->get_parameter("explorationBT.bt_xml_path").as_string();
  parameter_map_["explorationBT.exploration_boundary"] =
    node->get_parameter("explorationBT.exploration_boundary").as_double_array();
  parameter_map_["explorationBT.abort_exploration_on_nav2_abort"] =
    node->get_parameter("explorationBT.abort_exploration_on_nav2_abort").as_bool();
  parameter_map_["explorationBT.increment_search_distance_by"] =
    node->get_parameter("explorationBT.increment_search_distance_by").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, "sensorSimulator.input_map_topic", rclcpp::ParameterValue(
      "/map"));
  nav2_util::declare_parameter_if_not_declared(
    node, "sensorSimulator.input_map_is_transient_local", rclcpp::ParameterValue(
      true));
  nav2_util::declare_parameter_if_not_declared(
    node, "sensorSimulator.explored_map_topic", rclcpp::ParameterValue(
      "/explored_map"));
  nav2_util::declare_parameter_if_not_declared(
    node, "sensorSimulator.angular_resolution", rclcpp::ParameterValue(
      0.013));
  nav2_util::declare_parameter_if_not_declared(
    node, "sensorSimulator.sensor_update_rate", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "sensorSimulator.sensor_min_angle", rclcpp::ParameterValue(
      -M_PI / 4));
  nav2_util::declare_parameter_if_not_declared(
    node, "sensorSimulator.sensor_max_angle", rclcpp::ParameterValue(
      M_PI / 4));
  nav2_util::declare_parameter_if_not_declared(
    node, "sensorSimulator.sensor_max_range", rclcpp::ParameterValue(
      3.0));

  parameter_map_["sensorSimulator.input_map_topic"] = node->get_parameter(
    "sensorSimulator.input_map_topic").as_string();
  parameter_map_["sensorSimulator.input_map_is_transient_local"] = node->get_parameter(
    "sensorSimulator.input_map_is_transient_local").as_bool();
  parameter_map_["sensorSimulator.explored_map_topic"] = node->get_parameter(
    "sensorSimulator.explored_map_topic").as_string();
  parameter_map_["sensorSimulator.angular_resolution"] = node->get_parameter(
    "sensorSimulator.angular_resolution").as_double();
  parameter_map_["sensorSimulator.sensor_update_rate"] = node->get_parameter(
    "sensorSimulator.sensor_update_rate").as_double();
  parameter_map_["sensorSimulator.sensor_min_angle"] = node->get_parameter(
    "sensorSimulator.sensor_min_angle").as_double();
  parameter_map_["sensorSimulator.sensor_max_angle"] = node->get_parameter(
    "sensorSimulator.sensor_max_angle").as_double();
  parameter_map_["sensorSimulator.sensor_max_range"] = node->get_parameter(
    "sensorSimulator.sensor_max_range").as_double();

  sanityCheckParameters();

  dynamic_param_callback_handle_ =
    node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::dynamicReconfigureCallback,
      this, std::placeholders::_1));
}

void ParameterHandler::makeParametersYAMLcpp()
{
  std::string yaml_path;
  YAML::Node yaml_node;
  yaml_path = roadmap_explorer_dir + "/params/exploration.yaml";

  // Load YAML file and retrieve parameters
  YAML::Node loaded_node = YAML::LoadFile(yaml_path);

  parameter_map_["frontierSearch.min_frontier_cluster_size"] =
    loaded_node["frontierSearch"]["min_frontier_cluster_size"].as<double>();
  parameter_map_["frontierSearch.max_frontier_cluster_size"] =
    loaded_node["frontierSearch"]["max_frontier_cluster_size"].as<double>();
  parameter_map_["frontierSearch.frontier_search_distance"] =
    loaded_node["frontierSearch"]["frontier_search_distance"].as<double>();
  parameter_map_["frontierSearch.lethal_threshold"] =
    loaded_node["frontierSearch"]["lethal_threshold"].as<int>();
  parameter_map_["frontierSearch.max_frontier_search_distance"] =
    loaded_node["frontierSearch"]["max_frontier_search_distance"].as<double>();

  parameter_map_["costCalculator.max_camera_depth"] =
    loaded_node["costCalculator"]["max_camera_depth"].as<double>();
  parameter_map_["costCalculator.delta_theta"] =
    loaded_node["costCalculator"]["delta_theta"].as<double>();
  parameter_map_["costCalculator.camera_fov"] =
    loaded_node["costCalculator"]["camera_fov"].as<double>();
  parameter_map_["costCalculator.factor_of_max_is_min"] =
    loaded_node["costCalculator"]["factor_of_max_is_min"].as<double>();
  parameter_map_["costCalculator.closeness_rejection_threshold"] =
    loaded_node["costCalculator"]["closeness_rejection_threshold"].as<double>();
  parameter_map_["costCalculator.planner_allow_unknown"] =
    loaded_node["costCalculator"]["planner_allow_unknown"].as<bool>();
  parameter_map_["costCalculator.max_planning_distance_roadmap"] =
    loaded_node["costCalculator"]["max_planning_distance_roadmap"].as<double>();

  parameter_map_["costAssigner.cost_calculation_methods"] =
    loaded_node["costAssigner"]["cost_calculation_methods"].as<std::vector<std::string>>();

  parameter_map_["frontierRoadmap.max_graph_reconstruction_distance"] =
    loaded_node["frontierRoadmap"]["max_graph_reconstruction_distance"].as<double>();
  parameter_map_["frontierRoadmap.grid_cell_size"] =
    loaded_node["frontierRoadmap"]["grid_cell_size"].as<double>();
  parameter_map_["frontierRoadmap.radius_to_decide_edges"] =
    loaded_node["frontierRoadmap"]["radius_to_decide_edges"].as<double>();
  parameter_map_["frontierRoadmap.min_distance_between_two_frontier_nodes"] =
    loaded_node["frontierRoadmap"]["min_distance_between_two_frontier_nodes"].as<double>();
  parameter_map_["frontierRoadmap.min_distance_between_robot_pose_and_node"] =
    loaded_node["frontierRoadmap"]["min_distance_between_robot_pose_and_node"].as<double>();

  parameter_map_["fullPathOptimizer.num_frontiers_in_local_area"] =
    loaded_node["fullPathOptimizer"]["num_frontiers_in_local_area"].as<double>();
  parameter_map_["fullPathOptimizer.local_frontier_search_radius"] =
    loaded_node["fullPathOptimizer"]["local_frontier_search_radius"].as<double>();
  parameter_map_["fullPathOptimizer.add_yaw_to_tsp"] =
    loaded_node["fullPathOptimizer"]["add_yaw_to_tsp"].as<bool>();
  parameter_map_["fullPathOptimizer.add_distance_to_robot_to_tsp"] =
    loaded_node["fullPathOptimizer"]["add_distance_to_robot_to_tsp"].as<bool>();

  parameter_map_["goalHysteresis.use_euclidean_distance"] =
    loaded_node["goalHysteresis"]["use_euclidean_distance"].as<bool>();
  parameter_map_["goalHysteresis.use_roadmap_planner_distance"] =
    loaded_node["goalHysteresis"]["use_roadmap_planner_distance"].as<bool>();

  parameter_map_["explorationBT.bt_sleep_ms"] =
    loaded_node["explorationBT"]["bt_sleep_ms"].as<int>();
  parameter_map_["explorationBT.nav2_bt_xml"] =
    loaded_node["explorationBT"]["nav2_bt_xml"].as<std::string>();
  parameter_map_["explorationBT.bt_xml_path"] =
    loaded_node["explorationBT"]["bt_xml_path"].as<std::string>();
  parameter_map_["explorationBT.exploration_boundary"] =
    loaded_node["explorationBT"]["exploration_boundary"].as<std::vector<double>>();
  parameter_map_["explorationBT.abort_exploration_on_nav2_abort"] =
    loaded_node["explorationBT"]["abort_exploration_on_nav2_abort"].as<bool>();
  parameter_map_["explorationBT.increment_search_distance_by"] =
    loaded_node["explorationBT"]["increment_search_distance_by"].as<double>();

  parameter_map_["sensorSimulator.input_map_topic"] =
    loaded_node["sensorSimulator"]["input_map_topic"].as<std::string>();
  parameter_map_["sensorSimulator.input_map_is_transient_local"] =
    loaded_node["sensorSimulator"]["input_map_is_transient_local"].as<bool>();
  parameter_map_["sensorSimulator.explored_map_topic"] =
    loaded_node["sensorSimulator"]["explored_map_topic"].as<std::string>();
  parameter_map_["sensorSimulator.angular_resolution"] =
    loaded_node["sensorSimulator"]["angular_resolution"].as<double>();
  parameter_map_["sensorSimulator.sensor_update_rate"] =
    loaded_node["sensorSimulator"]["sensor_update_rate"].as<double>();
  parameter_map_["sensorSimulator.sensor_min_angle"] =
    loaded_node["sensorSimulator"]["sensor_min_angle"].as<double>();
  parameter_map_["sensorSimulator.sensor_max_angle"] =
    loaded_node["sensorSimulator"]["sensor_max_angle"].as<double>();
  parameter_map_["sensorSimulator.sensor_max_range"] =
    loaded_node["sensorSimulator"]["sensor_max_range"].as<double>();

  sanityCheckParameters();
}

void ParameterHandler::sanityCheckParameters()
{
  if (getValue<bool>("goalHysteresis.use_euclidean_distance") == true &&
    getValue<bool>("goalHysteresis.use_roadmap_planner_distance") == true)
  {
    throw RoadmapExplorerException(
            "Both use_euclidean_distance and use_roadmap_planner_distance are set to true. Please set only one of them to true.");
  } else if (getValue<bool>("goalHysteresis.use_euclidean_distance") == false &&
    getValue<bool>("goalHysteresis.use_roadmap_planner_distance") == false)
  {
    throw RoadmapExplorerException(
            "Both use_euclidean_distance and use_roadmap_planner_distance are set to false. Please set only one of them to true.");
  }

  if (getValue<int64_t>("frontierSearch.lethal_threshold") > 255 ||
    getValue<int64_t>("frontierSearch.lethal_threshold") < 0)
  {
    throw RoadmapExplorerException("Lethal thresholds out of unsigned char range.");
  }
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::dynamicReconfigureCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  // make a copy of the parameter_map_ to revert in case of failure
  auto parameter_map_copy_ = parameter_map_;

  // 1) Apply each incoming parameter
  for (const auto & param : parameters) {
    const auto & name = param.get_name();

    // Only accept parameters we declared
    if (parameter_map_.find(name) == parameter_map_.end()) {
      result.successful = false;
      result.reason = "Unknown parameter: " + name;
      return result;
    }

    // 2) Update parameter_map_ based on type
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        parameter_map_[name] = param.as_double();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        parameter_map_[name] = param.as_int();
        break;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        parameter_map_[name] = param.as_bool();
        break;
      default:
        result.successful = false;
        result.reason = "Unsupported type for parameter: " + name;
        return result;
    }
  }

  try {
    sanityCheckParameters();
  } catch (const RoadmapExplorerException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  if (!result.successful) {
    // revert to the previous state if sanity check fails
    LOG_ERROR("Parameter update: Sanity check failed: " + result.reason);
    parameter_map_ = parameter_map_copy_;
  }
  return result;
}
