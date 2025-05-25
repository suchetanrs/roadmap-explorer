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

void ParameterHandler::makeParameters(bool use_ros_parameters, rclcpp::Node::SharedPtr node)
{
  if (use_ros_parameters) {
    makeParametersROS(node);
  } else {
    makeParametersYAMLcpp();
  }
}

void ParameterHandler::makeParametersROS(rclcpp::Node::SharedPtr node)
{
  // --- frontierSearch ---
  node->declare_parameter<double>("frontierSearch.min_frontier_cluster_size", 1.0);
  node->declare_parameter<double>("frontierSearch.max_frontier_cluster_size", 20.0);
  node->declare_parameter<double>("frontierSearch.max_frontier_distance", 50.0);
  node->declare_parameter<int64_t>("frontierSearch.lethal_threshold", 160);

  parameter_map_["frontierSearch.min_frontier_cluster_size"] = node->get_parameter(
    "frontierSearch.min_frontier_cluster_size").as_double();
  parameter_map_["frontierSearch.max_frontier_cluster_size"] = node->get_parameter(
    "frontierSearch.max_frontier_cluster_size").as_double();
  parameter_map_["frontierSearch.max_frontier_distance"] = node->get_parameter(
    "frontierSearch.max_frontier_distance").as_double();
  parameter_map_["frontierSearch.lethal_threshold"] = node->get_parameter(
    "frontierSearch.lethal_threshold").as_int();

  // --- costCalculator ---
  node->declare_parameter<double>("costCalculator.max_camera_depth", 2.0);
  node->declare_parameter<double>("costCalculator.delta_theta", 0.10);
  node->declare_parameter<double>("costCalculator.camera_fov", 1.04);
  node->declare_parameter<double>("costCalculator.factor_of_max_is_min", 0.70);
  node->declare_parameter<double>("costCalculator.closeness_rejection_threshold", 0.5);
  node->declare_parameter<bool>("costCalculator.planner_allow_unknown", true);

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

  // --- costAssigner ---
  std::vector<std::string> default_cost_calculation_methods =
  {"RoadmapPlannerDistance", "ArrivalInformation"};
  // {"A*PlannerDistance", "ArrivalInformation"};
  // {"EuclideanDistance", "ArrivalInformation"};
  // {"RandomCosts"};
  // {};
  node->declare_parameter<std::vector<std::string>>(
    "costAssigner.cost_calculation_methods",
    default_cost_calculation_methods);

  parameter_map_["costAssigner.cost_calculation_methods"] =
    node->get_parameter("costAssigner.cost_calculation_methods").as_string_array();

  // --- frontierRoadmap ---
  node->declare_parameter<double>("frontierRoadmap.max_graph_reconstruction_distance", 25.0);
  node->declare_parameter<double>("frontierRoadmap.grid_cell_size", 1.0);
  node->declare_parameter<double>("frontierRoadmap.radius_to_decide_edges", 6.1);
  node->declare_parameter<double>("frontierRoadmap.min_distance_between_two_frontier_nodes", 0.25);
  node->declare_parameter<double>("frontierRoadmap.min_distance_between_robot_pose_and_node", 0.25);

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
  node->declare_parameter<double>("fullPathOptimizer.num_frontiers_in_local_area", 5.0);
  node->declare_parameter<double>("fullPathOptimizer.local_frontier_search_radius", 12.0);
  node->declare_parameter<bool>("fullPathOptimizer.add_yaw_to_tsp", false);
  node->declare_parameter<bool>("fullPathOptimizer.add_distance_to_robot_to_tsp", false);

  parameter_map_["fullPathOptimizer.num_frontiers_in_local_area"] = node->get_parameter(
    "fullPathOptimizer.num_frontiers_in_local_area").as_double();
  parameter_map_["fullPathOptimizer.local_frontier_search_radius"] = node->get_parameter(
    "fullPathOptimizer.local_frontier_search_radius").as_double();
  parameter_map_["fullPathOptimizer.add_yaw_to_tsp"] = node->get_parameter(
    "fullPathOptimizer.add_yaw_to_tsp").as_bool();
  parameter_map_["fullPathOptimizer.add_distance_to_robot_to_tsp"] = node->get_parameter(
    "fullPathOptimizer.add_distance_to_robot_to_tsp").as_bool();

  // --- goalHysteresis ---
  node->declare_parameter<bool>("goalHysteresis.use_euclidean_distance", false);
  node->declare_parameter<bool>("goalHysteresis.use_roadmap_planner_distance", true);

  parameter_map_["goalHysteresis.use_euclidean_distance"] = node->get_parameter(
    "goalHysteresis.use_euclidean_distance").as_bool();
  parameter_map_["goalHysteresis.use_roadmap_planner_distance"] = node->get_parameter(
    "goalHysteresis.use_roadmap_planner_distance").as_bool();

  // --- explorationBT ---
  node->declare_parameter<int64_t>("explorationBT.bt_sleep_ms", 70);
  node->declare_parameter<std::string>("explorationBT.nav2_bt_xml", roadmap_explorer_dir + "/xml/explore_to_pose.xml");
  node->declare_parameter<std::string>("explorationBT.bt_xml_path", roadmap_explorer_dir + "/xml/exploration.xml");
  std::vector default_exploration_boundary = {310.0, 260.0, 310.0, -120.0, -70.0, -120.0, -70.0, 260.0};
  node->declare_parameter<std::vector<double>>("explorationBT.exploration_boundary", default_exploration_boundary);

  parameter_map_["explorationBT.bt_sleep_ms"] =
    node->get_parameter("explorationBT.bt_sleep_ms").as_int();
  parameter_map_["explorationBT.nav2_bt_xml"] =
    node->get_parameter("explorationBT.nav2_bt_xml").as_string();
  parameter_map_["explorationBT.bt_xml_path"] =
    node->get_parameter("explorationBT.bt_xml_path").as_string();
  parameter_map_["explorationBT.exploration_boundary"] =
    node->get_parameter("explorationBT.exploration_boundary").as_double_array();

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
  parameter_map_["frontierSearch.max_frontier_distance"] =
    loaded_node["frontierSearch"]["max_frontier_distance"].as<double>();
  parameter_map_["frontierSearch.lethal_threshold"] =
    loaded_node["frontierSearch"]["lethal_threshold"].as<int>();

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

  sanityCheckParameters();
}

void ParameterHandler::sanityCheckParameters()
{
  if (getValue<bool>("goalHysteresis.use_euclidean_distance") == true &&
    getValue<bool>("goalHysteresis.use_roadmap_planner_distance") == true)
  {
    throw std::runtime_error(
            "Both use_euclidean_distance and use_roadmap_planner_distance are set to true. Please set only one of them to true.");
  } else if (getValue<bool>("goalHysteresis.use_euclidean_distance") == false &&
    getValue<bool>("goalHysteresis.use_roadmap_planner_distance") == false)
  {
    throw std::runtime_error(
            "Both use_euclidean_distance and use_roadmap_planner_distance are set to false. Please set only one of them to true.");
  }

  if (getValue<int64_t>("frontierSearch.lethal_threshold") > 255 ||
    getValue<int64_t>("frontierSearch.lethal_threshold") < 0)
  {
    throw std::runtime_error("Lethal thresholds out of unsigned char range.");
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
  } catch (const std::exception & e) {
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
