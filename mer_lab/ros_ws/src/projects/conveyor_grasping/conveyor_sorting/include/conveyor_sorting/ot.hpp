#ifndef CONVEYOR_SORTING_OT_HPP
#define CONVEYOR_SORTING_OT_HPP

#include <string>
#include <vector>

#include <ros/ros.h>

#include <qnd/comm.hpp>

#include "conveyor_sorting/OT.h"
#include "conveyor_sorting/types.hpp"

namespace conveyor_sorting {

  class NodeOTCalculator {
  public:
    NodeOTCalculator(ros::NodeHandle& nh, const std::string& otTopic="compute_optimal_transport")
      : _otClient{*qnd::serviceClient<OT>(nh, otTopic)} {}

    AssociatedPoints operator()(const UnassociatedPoints& pts,
				const std::vector<double>& costMatrix) {
      AssociatedPoints ret{};

      OT otMsg{};

      otMsg.request.source = pts.first;
      otMsg.request.target = pts.second;
      otMsg.request.cost = costMatrix;

      if(!_otClient.call(otMsg))
	ROS_FATAL("Failed to get optimal transport solution from client");

      // Convert association into an object
      for(std::size_t i = 0; i < pts.first.size(); ++i)
	ret.push_back({pts.first[i], pts.second[otMsg.response.association[i]]});

      // All done
      return ret;
    }
  private:
    ros::ServiceClient _otClient;
  };

} // conveyor_sorting

#endif
