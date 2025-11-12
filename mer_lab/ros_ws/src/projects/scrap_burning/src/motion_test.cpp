#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "descartes_service_planner/ExecuteTrajectory.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_test_node");
  ros::NodeHandle nh;

  if(argc != 8) {
    ROS_ERROR_STREAM("ERROR: Please provide input pose: 'x y z o_x o_y o_z o_w'");
    return 1;
  }

  ros::ServiceClient motion_client
    = nh.serviceClient<descartes_service_planner::ExecuteTrajectory>("execute_cartesian_trajectory");

  descartes_service_planner::ExecuteTrajectory traj;
  geometry_msgs::Pose target;
  target.position.x = std::atof(argv[1]);
  target.position.y = std::atof(argv[2]);
  target.position.z = std::atof(argv[3]);
  target.orientation.x = std::atof(argv[4]);
  target.orientation.y = std::atof(argv[5]);
  target.orientation.z = std::atof(argv[6]);
  target.orientation.w = std::atof(argv[7]);

  // Insert the target twice to ensure descartes works fine
  traj.request.poses.push_back(target);
  // target.position.z -= 0.05;
  traj.request.poses.push_back(target);
  traj.request.start_time = 5.0;
  // Keep delta_time, tolerances = 0
  traj.request.control_topic = "/panda/gazebo_ros_control";
  traj.request.group_name = "arm";
  traj.request.base_link = "world";
  traj.request.eef_link = "end_effector_link";

  return !motion_client.call(traj);
}
