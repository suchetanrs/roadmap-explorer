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
    outFile << ++serialNumber << "," << "planning_iteration" << "," << planningCount << "\n";
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
  std::string csvFilename;
  bool logToCSV;
  int serialNumber;
  std::string baseFilename;
  std::mutex mapMutex;
  int planningCount;
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
  std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

#define PROFILE_FUNCTION Profiler profiler_instance(__func__);
#endif // EVENT_LOGGER_HPP_
