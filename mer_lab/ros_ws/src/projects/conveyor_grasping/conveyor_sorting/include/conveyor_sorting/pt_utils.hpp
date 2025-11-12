#ifndef CONVEYOR_SORTING_PT_UTILS_HPP
#define CONVEYOR_SORTING_PT_UTILS_HPP

#include <cmath>

#include <geometry_msgs/Point.h>

geometry_msgs::Point createUnitVector(double angle) {
  geometry_msgs::Point ret;

  ret.x = std::cos(angle);
  ret.y = std::sin(angle);
  ret.z = 0.0;

  return ret;
}

#endif
