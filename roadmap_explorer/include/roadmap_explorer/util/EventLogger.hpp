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

#ifndef EVENT_LOGGER_HPP_
#define EVENT_LOGGER_HPP_

#include <iostream>
#include <unordered_map>
#include <chrono>
#include <string>
#include <fstream>
#include <thread>
#include <mutex>
#include <random>
#include <iomanip>
#include <sstream>
#include <ctime>

#include "roadmap_explorer/util/Logger.hpp"

class EventLogger
{
public:
  ~EventLogger();

  static void createInstance(bool logToCSV = false)
  {
    LOG_INFO("Creating event logger instance");
    std::lock_guard<std::mutex> lock(instanceMutex_);
    if (EventLoggerPtr_ == nullptr) {
      EventLoggerPtr_.reset(new EventLogger(logToCSV));
    } else {
      throw RoadmapExplorerException("EventLogger instance already exists!");
    }
  }

  static EventLogger & getInstance()
  {
    std::lock_guard<std::mutex> lock(instanceMutex_);
    if (EventLoggerPtr_ == nullptr) {
      throw RoadmapExplorerException("Cannot dereference a null EventLogger! :(");
    }
    return *EventLoggerPtr_;
  }

  static void destroyInstance()
  {
    LOG_INFO("EventLogger::destroyInstance()");
    EventLoggerPtr_.reset();
  }

  void startEvent(const std::string & key);

  /**
   * eventLevel:
   * 0 - MODULE LEVEL
   * 1 - SUBMODULE_LEVEL
   * 2 - EVENT LEVEL
   */
  void endEvent(const std::string & key, int eventLevel);

  /**
   * Returns in seconds
   */
  double getTimeSinceStart(const std::string & key);

  void incrementPlanningCount()
  {
    planningCount++;
    std::ofstream outFile(csvFilename, std::ios::out | std::ios::app);
    if (!outFile) {
      outFile << ++serialNumber << "," << "planning_iteration" << "," << planningCount << "\n";
    }
  }

  int getPlanningCount() const
  {
    return planningCount;
  }

private:
  // Delete copy constructor and assignment operator to prevent copying
  EventLogger(const EventLogger &) = delete;
  EventLogger & operator=(const EventLogger &) = delete;
  EventLogger(bool logToCSV);

  static std::unique_ptr<EventLogger> EventLoggerPtr_;
  static std::mutex instanceMutex_;
  std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> startTimes;
  int serialNumber;
  std::string baseFilename;
  std::string csvFilename;
  bool logToCSV;
  int planningCount;
  std::mutex mapMutex;
};

#define EventLoggerInstance (EventLogger::getInstance())

class Profiler
{
public:
  Profiler(const std::string & functionName)
  : functionName(functionName)
  {
    EventLoggerInstance.startEvent(functionName + "_profiler");
  }

  ~Profiler()
  {
    EventLoggerInstance.endEvent(functionName + "_profiler", 2);
  }

private:
  std::string functionName;
};

#define PROFILE_FUNCTION Profiler profiler_instance(__func__);
#endif // EVENT_LOGGER_HPP_
