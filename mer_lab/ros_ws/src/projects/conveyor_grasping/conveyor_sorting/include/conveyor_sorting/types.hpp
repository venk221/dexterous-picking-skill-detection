#ifndef CONVEYOR_SORTING_TYPES_HPP
#define CONVEYOR_SORTING_TYPES_HPP

#include <vector>

#include <geometry_msgs/Point.h>

namespace conveyor_sorting {

  using Point = std::pair<std::size_t, std::size_t>;
  using Region = std::pair<double, std::vector<Point> >;
  using PointPair = std::pair<geometry_msgs::Point, geometry_msgs::Point>;
  using AssociatedPoints = std::vector<PointPair>;
  using UnassociatedPoints = std::pair<std::vector<geometry_msgs::Point>,
				       std::vector<geometry_msgs::Point> >;
  using Size = Point;

} // conveyor_sorting

#endif
