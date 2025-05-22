#include <roadmap_explorer/Parameters.hpp>

std::unique_ptr<ParameterHandler> ParameterHandler::parameterHandlerPtr_ = nullptr;
std::recursive_mutex ParameterHandler::instanceMutex_;

ParameterHandler::ParameterHandler()
{
}

ParameterHandler::~ParameterHandler()
{
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

  parameter_map_["costCalculator.max_camera_depth"] = node->get_parameter(
    "costCalculator.max_camera_depth").as_double();
  parameter_map_["costCalculator.delta_theta"] =
    node->get_parameter("costCalculator.delta_theta").as_double();
  parameter_map_["costCalculator.camera_fov"] =
    node->get_parameter("costCalculator.camera_fov").as_double();
  parameter_map_["costCalculator.factor_of_max_is_min"] = node->get_parameter(
    "costCalculator.factor_of_max_is_min").as_double();

  // --- costAssigner ---
  node->declare_parameter<bool>("costAssigner.planner_allow_unknown", true);
  node->declare_parameter<bool>("costAssigner.add_heading_cost", true);

  parameter_map_["costAssigner.planner_allow_unknown"] = node->get_parameter(
    "costAssigner.planner_allow_unknown").as_bool();
  parameter_map_["costAssigner.add_heading_cost"] = node->get_parameter(
    "costAssigner.add_heading_cost").as_bool();

  // --- frontierRoadmap ---
  node->declare_parameter<double>("frontierRoadmap.max_frontier_distance", 25.0);
  node->declare_parameter<double>("frontierRoadmap.grid_cell_size", 1.0);
  node->declare_parameter<double>("frontierRoadmap.radius_to_decide_edges", 6.1);
  node->declare_parameter<double>("frontierRoadmap.min_distance_between_two_frontier_nodes", 0.25);
  node->declare_parameter<double>("frontierRoadmap.min_distance_between_robot_pose_and_node", 0.25);

  parameter_map_["frontierRoadmap.max_frontier_distance"] = node->get_parameter(
    "frontierRoadmap.max_frontier_distance").as_double();
  parameter_map_["frontierRoadmap.grid_cell_size"] = node->get_parameter(
    "frontierRoadmap.grid_cell_size").as_double();
  parameter_map_["frontierRoadmap.radius_to_decide_edges"] = node->get_parameter(
    "frontierRoadmap.radius_to_decide_edges").as_double();
  parameter_map_["frontierRoadmap.min_distance_between_two_frontier_nodes"] = node->get_parameter(
    "frontierRoadmap.min_distance_between_two_frontier_nodes").as_double();
  parameter_map_["frontierRoadmap.min_distance_between_robot_pose_and_node"] = node->get_parameter(
    "frontierRoadmap.min_distance_between_robot_pose_and_node").as_double();

  // --- goalHysteresis ---
  node->declare_parameter<bool>("goalHysteresis.use_euclidean_distance", false);
  node->declare_parameter<bool>("goalHysteresis.use_roadmap_planner_distance", true);

  parameter_map_["goalHysteresis.use_euclidean_distance"] = node->get_parameter(
    "goalHysteresis.use_euclidean_distance").as_bool();
  parameter_map_["goalHysteresis.use_roadmap_planner_distance"] = node->get_parameter(
    "goalHysteresis.use_roadmap_planner_distance").as_bool();

  // --- explorationBT ---
  node->declare_parameter<int64_t>("explorationBT.bt_sleep_ms", 70);

  parameter_map_["explorationBT.bt_sleep_ms"] =
    node->get_parameter("explorationBT.bt_sleep_ms").as_int();

  sanityCheckParameters();

  dynamic_param_callback_handle_ =
    node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::dynamicReconfigureCallback,
      this, std::placeholders::_1));
}

void ParameterHandler::makeParametersYAMLcpp()
{
  const std::string yaml_base_path =
    ament_index_cpp::get_package_share_directory("roadmap_explorer");
  std::string yaml_path;
  YAML::Node yaml_node;
  yaml_path = yaml_base_path + "/params/exploration.yaml";

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

  parameter_map_["costAssigner.planner_allow_unknown"] =
    loaded_node["costAssigner"]["planner_allow_unknown"].as<bool>();
  parameter_map_["costAssigner.add_heading_cost"] =
    loaded_node["costAssigner"]["add_heading_cost"].as<bool>();

  parameter_map_["frontierRoadmap.max_frontier_distance"] =
    loaded_node["frontierRoadmap"]["max_frontier_distance"].as<double>();
  parameter_map_["frontierRoadmap.grid_cell_size"] =
    loaded_node["frontierRoadmap"]["grid_cell_size"].as<double>();
  parameter_map_["frontierRoadmap.radius_to_decide_edges"] =
    loaded_node["frontierRoadmap"]["radius_to_decide_edges"].as<double>();
  parameter_map_["frontierRoadmap.min_distance_between_two_frontier_nodes"] =
    loaded_node["frontierRoadmap"]["min_distance_between_two_frontier_nodes"].as<double>();
  parameter_map_["frontierRoadmap.min_distance_between_robot_pose_and_node"] =
    loaded_node["frontierRoadmap"]["min_distance_between_robot_pose_and_node"].as<double>();

  parameter_map_["goalHysteresis.use_euclidean_distance"] =
    loaded_node["goalHysteresis"]["use_euclidean_distance"].as<bool>();
  parameter_map_["goalHysteresis.use_roadmap_planner_distance"] =
    loaded_node["goalHysteresis"]["use_roadmap_planner_distance"].as<bool>();

  parameter_map_["explorationBT.bt_sleep_ms"] =
    loaded_node["explorationBT"]["bt_sleep_ms"].as<int>();

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
    parameter_map_ = parameter_map_copy_;
  }
  return result;
}
