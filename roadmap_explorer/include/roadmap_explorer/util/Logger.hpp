#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <unordered_map>

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

#define LOG_LEVEL 3
#define TIME_LEVEL 0
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

////////////////////////////////////////////////////////////////////////////////////////

#define LOG_TRACE(X) \
  if (LOG_LEVEL >= 5) \
  std::cout << "\e[0;96m" << "[TRACE " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m" << std::endl;
#define LOG_DEBUG(X) \
  if (LOG_LEVEL >= 4) \
  std::cout << "\e[0;32m" << "[DEBUG " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m" << std::endl;
#define LOG_DEBUG_N(X) \
  if (LOG_LEVEL >= 4) \
  std::cout << "\e[0;32m" << "[DEBUG " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m";
#define LOG_INFO(X) \
  if (LOG_LEVEL >= 3) \
  std::cout << "[INFO " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << std::endl;
#define LOG_WARN(X) \
  if (LOG_LEVEL >= 2) \
  std::cout << "\e[0;93m" << "[WARN " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m" << std::endl;
#define LOG_ERROR(X) \
  if (LOG_LEVEL >= 1) \
  std::cout << "\e[0;31m" << "[ERROR " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m" << std::endl;
#define LOG_CRITICAL(X) \
  if (LOG_LEVEL >= 1) \
  std::cout << "\e[04;91m" << "[CRITICAL " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m" << std::endl;
#define LOG_FATAL(X) \
  if (LOG_LEVEL >= 0) \
  std::cout << "\e[1;37;41m" << "[FATAL " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m" << std::endl;

#define LOG_HIGHLIGHT(X) \
  std::cout << "\e[1;37;102m" << "[HIGHLIGHT " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m" << std::endl;

#define LOG_FLOW(X) \
  if (USE_MODULE_FLOW) \
  std::cout << "\e[1;37;103m" << "[FLOW " << std::chrono::system_clock::to_time_t( \
    std::chrono::high_resolution_clock::now()) << "]  " << X << "\e[m" << std::endl;

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
#endif // COLOR_H
