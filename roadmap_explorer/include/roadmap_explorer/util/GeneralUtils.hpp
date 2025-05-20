#ifndef COLOR_HPP_
#define COLOR_HPP_

#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

template<typename T>
inline bool vectorContains(const std::vector<T> & vec, const T & value)
{
  return std::find(vec.begin(), vec.end(), value) != vec.end();
}

#endif // COLOR_H
