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
      throw std::runtime_error("Parameter " + parameterKey + " is not found in the map");
    }
  }

  void makeParameters(bool use_ros_parameters, std::shared_ptr<nav2_util::LifecycleNode> node);

  static ParameterHandler & getInstance()
  {
    if (parameterHandlerPtr_ == nullptr) {
      parameterHandlerPtr_.reset(new ParameterHandler());
    }
    return *parameterHandlerPtr_;
  }

  void cleanupInstance()
  {
    LOG_INFO("ParameterHandler::cleanupInstance()");
    dynamic_param_callback_handle_.reset();
  }

private:
  void makeParametersROS(std::shared_ptr<nav2_util::LifecycleNode> node);

  void makeParametersYAMLcpp();

  void sanityCheckParameters();

  rcl_interfaces::msg::SetParametersResult  dynamicReconfigureCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  template<typename T>
  void setValue(const std::string & parameterKey, const T & value)
  {
    std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
    parameter_map_[parameterKey] = value;
  }

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

inline ParameterHandler & parameterInstance = ParameterHandler::getInstance();

#endif
