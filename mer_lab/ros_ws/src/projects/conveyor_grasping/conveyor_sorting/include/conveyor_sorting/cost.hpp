#ifndef CONVEYOR_SORTING_COST_HPP
#define CONVEYOR_SORTING_COST_HPP

#include <vector>

#include <geometry_msgs/Point.h>

#include <qnd/geom.hpp>

namespace conveyor_sorting {

  struct DistanceCost {
    std::vector<double> operator()(const UnassociatedPoints& uPts) {
      const std::vector<geometry_msgs::Point>& src = uPts.first;
      const std::vector<geometry_msgs::Point>& tgt = uPts.second;

      std::vector<double> ret(src.size() * tgt.size());

      for(std::size_t sourceIdx = 0; sourceIdx < src.size(); ++sourceIdx)
	for(std::size_t targetIdx = 0; targetIdx < tgt.size(); ++targetIdx)
	  ret[sourceIdx * tgt.size() + targetIdx] = qnd::distance(src[sourceIdx], tgt[targetIdx]);

      return ret;
    }
  };

} // conveyor_sorting

#endif
