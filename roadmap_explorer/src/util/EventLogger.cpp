#include <roadmap_explorer/util/EventLogger.hpp>

std::unique_ptr<EventLogger> EventLogger::EventLoggerPtr_ = nullptr;
std::mutex EventLogger::instanceMutex_;

EventLogger::EventLogger()
: logToCSV(false), serialNumber(0), baseFilename("event_log")
{

  if (logToCSV) {
    // Generate a unique filename by appending a timestamp and a random number
    auto now = std::chrono::system_clock::now();
    auto nowTime = std::chrono::system_clock::to_time_t(now);
    auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
      1000;

    std::ostringstream oss;
    oss << baseFilename << "_" <<
      std::put_time(std::localtime(&nowTime), "%Y%m%d_%H%M%S") << "_" << nowMs.count() << ".csv";
    csvFilename = oss.str();

    // Open the new CSV file and write the header
    std::ofstream outFile(csvFilename, std::ios::out | std::ios::app);
    if (!outFile) {
      throw RoadmapExplorerException("Unable to open file: " + csvFilename);
    }
    outFile << "SerialNumber,Event,Duration(seconds)\n";
  }
}

EventLogger::~EventLogger()
{
  LOG_INFO("EventLogger::~EventLogger()");
  if (logToCSV) {
    LOG_INFO("EventLogger: Closing CSV file " << csvFilename);
    // Close the CSV file if it was opened
    std::ofstream outFile(csvFilename, std::ios::out | std::ios::app);
    outFile.close();
  }
  startTimes.clear();
}

// Function to start an event
void EventLogger::startEvent(const std::string & key)
{
  std::lock_guard<std::mutex> lock(mapMutex);
  auto startTime = std::chrono::high_resolution_clock::now();
  startTimes[key] = startTime;
}

// Function to end an event and log the total time taken
void EventLogger::endEvent(const std::string & key, int eventLevel)
{
  auto endTime = std::chrono::high_resolution_clock::now();
  std::lock_guard<std::mutex> lock(mapMutex);
  if (startTimes.find(key) != startTimes.end()) {
    auto startTime = startTimes[key];
    std::chrono::duration<double> duration = endTime - startTime;
    if (eventLevel == -1) {
      LOG_ITERATION_TIME(key, duration.count());
    } else if (eventLevel == 0) {
      LOG_MODULE_TIME(key, duration.count());
    } else if (eventLevel == 1) {
      LOG_SUBMODULE_TIME(key, duration.count());
    } else if (eventLevel == 2) {
      LOG_EVENT_TIME(key, duration.count());
    } else {
      LOG_CRITICAL("eventLevel undefined for " << key);
    }

    // Write to CSV file
    if (logToCSV) {
      std::ofstream outFile(csvFilename, std::ios::out | std::ios::app);
      outFile << ++serialNumber << "," << key << "," << duration.count() << "\n";
    }

    startTimes.erase(key);
  } else {
    LOG_CRITICAL("Event " << key << " is not started");
  }
}

double EventLogger::getTimeSinceStart(const std::string & key)
{
  auto endTime = std::chrono::high_resolution_clock::now();
  std::lock_guard<std::mutex> lock(mapMutex);
  if (startTimes.find(key) != startTimes.end()) {
    auto startTime = startTimes[key];
    std::chrono::duration<double> duration = endTime - startTime;
    return duration.count();
  } else {
    LOG_CRITICAL("Event " << key << " is not started");
  }
  return 0.0;
}
