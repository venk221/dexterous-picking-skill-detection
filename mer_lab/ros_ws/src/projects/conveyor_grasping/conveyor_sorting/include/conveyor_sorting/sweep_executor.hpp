#ifndef CONVEYOR_SORTING_SWEEP_EXECUTOR_HPP
#define CONVEYOR_SORTING_SWEEP_EXECUTOR_HPP

#include <ros/ros.h>
#include <qnd/comm.hpp>

#include "conveyor_sorting_msgs/Sweep.h"
#include "conveyor_sorting_msgs/SweepingAction.h"

namespace conveyor_sorting {

  class SweepExecutor {
  public:
    SweepExecutor(ros::NodeHandle& nh, const std::string& sweepTopic, ros::Duration timeout=qnd::indefinite)
      : _sweepClient{} {
      std::optional<ros::ServiceClient> sweepClient(qnd::serviceClient<conveyor_sorting_msgs::Sweep>(nh, sweepTopic, timeout));
      if(!sweepClient)
	ROS_FATAL("Failed to connect to sweeping server");

      _sweepClient = *sweepClient;
    }

    bool operator()(const conveyor_sorting_msgs::SweepingAction& action, bool execute=true) {
      conveyor_sorting_msgs::Sweep msg{};
      msg.request.sweeping_action = action;
      msg.request.scale = 1.0;
      msg.request.execute = execute;
      return _sweepClient.call(msg);
    }
  private:
    ros::ServiceClient _sweepClient;
  };

} // conveyor_sorting

#endif
