#ifndef PARAMETER_HPP_
#define PARAMETER_HPP_

#include <string>
#include <fstream>
#include <mutex>
#include <random>
#include <ctime>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_util/node_utils.hpp>

#include "roadmap_explorer/util/Logger.hpp"

class ParameterHandler
{
public:
  ~ParameterHandler();

  template<typename T>
  T getValue(std::string parameterKey)
  {
    std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
    if (parameter_map_.find(parameterKey) != parameter_map_.end()) {
      // LOG_HIGHLIGHT("Got request for: " << parameterKey);
      // LOG_HIGHLIGHT("Returning value " << boost::any_cast<T>(parameter_map_[parameterKey]) << " for parameter " << parameterKey);
      return boost::any_cast<T>(parameter_map_[parameterKey]);
    } else {
      // TODO(suchetanrs) : Handle this runtime error.
      throw RoadmapExplorerException("Parameter " + parameterKey + " is not found in the map");
    }
  }

  void makeParameters(std::shared_ptr<nav2_util::LifecycleNode> node);

  static void createInstance()
  {
    LOG_INFO("Creating ParameterHandler instance");
    if (parameterHandlerPtr_ == nullptr) {
      parameterHandlerPtr_.reset(new ParameterHandler());
    }
    else
    {
      throw RoadmapExplorerException("ParameterHandler already exists!");
    }
  }

  static ParameterHandler & getInstance()
  {
    if (parameterHandlerPtr_ == nullptr) {
      throw RoadmapExplorerException("Cannot de-reference a null ParameterHandler! :(");
    }
    return *parameterHandlerPtr_;
  }

  static void destroyInstance()
  {
    LOG_INFO("ParameterHandler::destroyInstance()");
    parameterHandlerPtr_.reset();
  }

  template<typename T>
  void setValue(const std::string & parameterKey, const T & value)
  {
    std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
    parameter_map_[parameterKey] = value;
  }

private:
  void sanityCheckParameters();

  rcl_interfaces::msg::SetParametersResult  dynamicReconfigureCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  ParameterHandler(const ParameterHandler &) = delete;
  ParameterHandler & operator=(const ParameterHandler &) = delete;
  ParameterHandler();
  static std::unique_ptr<ParameterHandler> parameterHandlerPtr_;
  static std::recursive_mutex instanceMutex_;
  std::unordered_map<std::string, boost::any> parameter_map_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dynamic_param_callback_handle_;
  const std::string roadmap_explorer_dir =
    ament_index_cpp::get_package_share_directory("roadmap_explorer");
};

#define parameterInstance (ParameterHandler::getInstance())

#endif
