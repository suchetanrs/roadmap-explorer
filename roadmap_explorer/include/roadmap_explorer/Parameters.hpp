#ifndef PARAMETER_HPP_
#define PARAMETER_HPP_

#include <iostream>
#include <unordered_map>
#include <chrono>
#include <string>
#include <fstream>
#include <thread>
#include <mutex>
#include <random>
#include <ctime>

#include <iomanip>
#include <sstream>
#include <ctime>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

class ParameterHandler
{
public:
  ParameterHandler();

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
      // TODO : Handle this runtime error.
      throw std::runtime_error("Parameter " + parameterKey + " is not found in the map");
    }
  }
  
  void makeParameters(bool use_ros_parameters, rclcpp::Node::SharedPtr node);

  static ParameterHandler & getInstance()
  {
    if (parameterHandlerPtr_ == nullptr) {
      parameterHandlerPtr_.reset(new ParameterHandler());
    }
    return *parameterHandlerPtr_;
  }

private:
  void makeParametersROS(rclcpp::Node::SharedPtr node);

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
  static std::unique_ptr<ParameterHandler> parameterHandlerPtr_;
  static std::recursive_mutex instanceMutex_;
  std::unordered_map<std::string, boost::any> parameter_map_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dynamic_param_callback_handle_;
};

inline ParameterHandler & parameterInstance = ParameterHandler::getInstance();

#endif
