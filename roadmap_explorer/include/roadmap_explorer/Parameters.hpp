/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

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

/**
 * @class ParameterHandler
 * @brief Singleton class for managing exploration parameters
 * 
 * This class handles parameter declaration, retrieval, and storage for the
 * roadmap explorer system. It provides thread-safe access to parameters
 * across different modules including frontier search, roadmap planning,
 * path optimization, behavior trees, and sensor simulation.
 */
class ParameterHandler
{
public:
  ~ParameterHandler();

  /**
   * @brief Get the value of a parameter by key
   * @tparam T The type of the parameter value
   * @param parameterKey The key identifying the parameter
   * @return The parameter value cast to type T
   * @throws RoadmapExplorerException if parameter key is not found
   */
  template<typename T>
  T getValue(std::string parameterKey)
  {
    std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
    if (parameter_map_.find(parameterKey) != parameter_map_.end()) {
      // LOG_HIGHLIGHT("Got request for: " << parameterKey);
      // LOG_HIGHLIGHT("Returning value " << boost::any_cast<T>(parameter_map_[parameterKey]) << " for parameter " << parameterKey);
      return boost::any_cast<T>(parameter_map_[parameterKey]);
    } else {
      throw RoadmapExplorerException("Parameter " + parameterKey + " is not found in the map");
    }
  }

  /**
   * @brief Declare and initialize all exploration parameters from ROS node
   * @param node Shared pointer to the lifecycle node
   */
  void makeParameters(std::shared_ptr<nav2_util::LifecycleNode> node);

  /**
   * @brief Create the singleton instance of ParameterHandler
   * @throws RoadmapExplorerException if instance already exists
   */
  static void createInstance()
  {
    std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
    LOG_INFO("Creating ParameterHandler instance");
    if (parameterHandlerPtr_ == nullptr) {
      parameterHandlerPtr_.reset(new ParameterHandler());
    }
    else
    {
      throw RoadmapExplorerException("ParameterHandler already exists");
    }
  }

  /**
   * @brief Get the singleton instance of ParameterHandler
   * @return Reference to the ParameterHandler instance
   * @throws RoadmapExplorerException if instance has not been created
   */
  static ParameterHandler & getInstance()
  {
    std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
    if (parameterHandlerPtr_ == nullptr) {
      throw RoadmapExplorerException("Cannot dereference a null ParameterHandler");
    }
    return *parameterHandlerPtr_;
  }

  /**
   * @brief Destroy the singleton instance
   */
  static void destroyInstance()
  {
    std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
    LOG_INFO("ParameterHandler::destroyInstance()");
    parameterHandlerPtr_.reset();
  }

  /**
   * @brief Set the value of a parameter
   * @tparam T The type of the parameter value
   * @param parameterKey The key identifying the parameter
   * @param value The new value for the parameter
   */
  template<typename T>
  void setValue(const std::string & parameterKey, const T & value)
  {
    std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
    parameter_map_[parameterKey] = value;
  }

private:
  /**
   * @brief Validate parameter values for consistency and safety
   */
  void sanityCheckParameters();

  /**
   * @brief Callback for dynamic parameter reconfiguration
   * @param parameters Vector of parameters to update
   * @return Result indicating success or failure with reason
   */
  rcl_interfaces::msg::SetParametersResult dynamicReconfigureCallback(
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
