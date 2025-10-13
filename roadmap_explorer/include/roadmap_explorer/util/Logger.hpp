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

#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>

// Color_id	    Color
//        1	    Aqua
//        2	    Green
//        3	    Blue
//        4	    Red
//        5	    Purple
//        6	    Yellow
//        7	    White
//        8	    Gray
//        A	    Light Green
//        0	    Black
//        9	    Light  Blue
//        B	    Light Aqua
//        C	    Light Red
//        D	    Light Purple
//        E	    Light Yellow
//        F	    Bright White

/*
Attributes	        Foreground color	   Background color

00 = normal         31 = red               40 = black
01 = bold           32 = green             41 = red
04 = underlined     33 = orange            42 = green
05 = blinking       34 = blue              43 = orange
07 = reversed       35 = purple            44 = blue
08 = concealed      36 = cyan              45 = purple
                    37 = grey              46 = cyan
                    90 = dark grey         47 = grey
                    91 = light red         100 = dark grey
                    92 = light green       101 = light red
                    93 = yellow            102 = light green
                    94 = light blue        103 = yellow
                    95 = light purple      104 = light blue
                    96 = turquoise         105 = light purple
                                           106 = turquoise
**/

// ======================== Logger Configuration ========================
// These macros control logging behavior at compile time.
// To override, define them before including this header or via compiler flags.
//
// LOG_LEVEL: Controls which log messages are compiled in
//   5 = TRACE   (most verbose - all messages)
//   4 = DEBUG   (debugging information)
//   3 = INFO    (general information)
//   2 = WARN    (warnings only) [DEFAULT]
//   1 = ERROR   (errors and critical messages only)
//   0 = FATAL   (only fatal errors)
//  -1 = HIDE   (hide all logs)
//
// TIME_LEVEL: Controls which timing logs are shown
//   2 = Show all timing logs (EVENT, SUBMODULE, MODULE, ITERATION)
//   1 = Show SUBMODULE, MODULE, and ITERATION timing logs
//   0 = Show MODULE and ITERATION timing logs only [DEFAULT]
//  -1 = Hide all timing logs 
//
// USE_MODULE_FLOW: Enable/disable module flow tracking logs
//   true  = Show module flow logs [DEFAULT]
//   false = Hide module flow logs
// ======================================================================

#define LOG_LEVEL 2
#define TIME_LEVEL -1
#define USE_MODULE_FLOW false

////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
std::ostream & operator<<(std::ostream & os, const std::vector<T> & vec)
{
  os << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    os << vec[i];
    if (i != vec.size() - 1) {
      os << ", ";
    }
  }
  os << "]";
  return os;
}

template<typename K, typename V>
std::ostream & operator<<(std::ostream & os, const std::unordered_map<K, V> & map)
{
  os << "{ Keys: ";
  bool first = true;
  for (const auto & pair : map) {
    if (!first) {
      os << ", ";
    }
    os << pair.first;
    // ;
    first = false;
  }
  os << "}";
  return os;
}

#ifdef USE_RCLCPP_LOGGER     // <-- compile-time switch
// -------- send to rclcpp --------
  #define RCLCPP_LOG_MESSAGE(level, msg) \
  { \
    std::ostringstream _os; \
    _os << msg; \
    RCLCPP_ ## level ## _STREAM( \
      rclcpp::get_logger("roadmap_explorer_node"), _os.str()); \
  }

  #define LOG_TRACE(X) \
  if (LOG_LEVEL >= 5) \
  RCLCPP_LOG_MESSAGE(INFO, X);
  #define LOG_DEBUG(X) \
  if (LOG_LEVEL >= 4) \
  RCLCPP_LOG_MESSAGE(INFO, X);
  #define LOG_DEBUG_N(X) \
  if (LOG_LEVEL >= 4) \
  RCLCPP_LOG_MESSAGE(INFO, X);
  #define LOG_INFO(X) \
  if (LOG_LEVEL >= 3) \
  RCLCPP_LOG_MESSAGE(INFO, X);
  #define LOG_WARN(X) \
  if (LOG_LEVEL >= 2) \
  RCLCPP_LOG_MESSAGE(WARN, X);
  #define LOG_ERROR(X) \
  if (LOG_LEVEL >= 1) \
  RCLCPP_LOG_MESSAGE(ERROR, X);
  #define LOG_CRITICAL(X) \
  if (LOG_LEVEL >= 1) \
  RCLCPP_LOG_MESSAGE(ERROR, X);
  #define LOG_FATAL(X) \
  if (LOG_LEVEL >= 0) \
  RCLCPP_LOG_MESSAGE(FATAL, X);

  #define LOG_HIGHLIGHT(X) \
  RCLCPP_LOG_MESSAGE(WARN, X);

  #define LOG_FLOW(X) \
  if (USE_MODULE_FLOW) \
  RCLCPP_LOG_MESSAGE(WARN, X);

////////////////////////////////////////////////////////////////////////////////////////

  #define LOG_ITERATION_TIME(eventName, seconds) \
  RCLCPP_LOG_MESSAGE( \
    INFO, "\033[0;34m" << " ITERATION : " << eventName << ": Execution Time: " << seconds << \
      " Seconds\033[0m");

  #define LOG_EVENT_TIME(eventName, seconds) \
  if (TIME_LEVEL >= 2) \
  RCLCPP_LOG_MESSAGE( \
    INFO, "\033[0;34m" << " EVENT: " << eventName << ": Execution Time: " << seconds << \
      " Seconds\033[0m");

  #define LOG_SUBMODULE_TIME(eventName, seconds) \
  if (TIME_LEVEL >= 1) \
  RCLCPP_LOG_MESSAGE( \
    INFO, "\033[0;37;44m" << " SUBMODULE: " << eventName << ": Execution Time: " << seconds << \
      " Seconds\033[0m");

  #define LOG_MODULE_TIME(eventName, seconds) \
  if (TIME_LEVEL >= 0) \
  RCLCPP_LOG_MESSAGE( \
    INFO, "\033[1;37;44m" << " MODULE: " << eventName << ": Execution Time: " << seconds << \
      " Seconds\033[0m");


#else


  #define LOG_TRACE(X) \
  if (LOG_LEVEL >= 5) \
  std::cout << "\033[0;96m" << "[TRACE " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m" << std::endl;
  #define LOG_DEBUG(X) \
  if (LOG_LEVEL >= 4) \
  std::cout << "\033[0;32m" << "[DEBUG " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m" << std::endl;
  #define LOG_DEBUG_N(X) \
  if (LOG_LEVEL >= 4) \
  std::cout << "\033[0;32m" << "[DEBUG " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m";
  #define LOG_INFO(X) \
  if (LOG_LEVEL >= 3) \
  std::cout << "[INFO " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << std::endl;
  #define LOG_WARN(X) \
  if (LOG_LEVEL >= 2) \
  std::cout << "\033[0;93m" << "[WARN " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m" << std::endl;
  #define LOG_ERROR(X) \
  if (LOG_LEVEL >= 1) \
  std::cout << "\033[0;31m" << "[ERROR " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m" << std::endl;
  #define LOG_CRITICAL(X) \
  if (LOG_LEVEL >= 1) \
  std::cout << "\033[04;91m" << "[CRITICAL " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m" << std::endl;
  #define LOG_FATAL(X) \
  if (LOG_LEVEL >= 0) \
  std::cout << "\033[1;37;41m" << "[FATAL " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m" << std::endl;

  #define LOG_HIGHLIGHT(X) \
  std::cout << "\033[1;37;102m" << "[HIGHLIGHT " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m" << std::endl;

  #define LOG_FLOW(X) \
  if (USE_MODULE_FLOW) \
  std::cout << "\033[1;37;103m" << "[FLOW " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\033[m" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////

  #define LOG_ITERATION_TIME(eventName, seconds) \
  std::cout << "\033[0;34m" << " ITERATION : " << eventName << ": Execution Time: " << seconds << \
    " Seconds\033[0m" << std::endl;

  #define LOG_EVENT_TIME(eventName, seconds) \
  if (TIME_LEVEL >= 2) \
  std::cout << "\033[0;34m" << " EVENT: " << eventName << ": Execution Time: " << seconds << \
    " Seconds\033[0m" << std::endl;

  #define LOG_SUBMODULE_TIME(eventName, seconds) \
  if (TIME_LEVEL >= 1) \
  std::cout << "\033[0;37;44m" << " SUBMODULE: " << eventName << ": Execution Time: " << seconds << \
    " Seconds\033[0m" << std::endl;

  #define LOG_MODULE_TIME(eventName, seconds) \
  if (TIME_LEVEL >= 0) \
  std::cout << "\033[1;37;44m" << " MODULE: " << eventName << ": Execution Time: " << seconds << \
    " Seconds\033[0m" << std::endl;
#endif

class RoadmapExplorerException : public std::runtime_error
{
public:
  explicit RoadmapExplorerException(const std::string & description)
  : std::runtime_error(description)
  {
    LOG_FATAL("RoadmapExplorerException: " << description);
  }
};

#endif // LOGGER_HPP_
