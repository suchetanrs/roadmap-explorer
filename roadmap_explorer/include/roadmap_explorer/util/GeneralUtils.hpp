#ifndef GENERALUTILS_HPP_
#define GENERALUTILS_HPP_

#include <iostream>
#include <sstream>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

template<typename T>
inline bool vectorContains(const std::vector<T> & vec, const T & value)
{
  return std::find(vec.begin(), vec.end(), value) != vec.end();
}

#endif // GENERALUTILS_HPP_
