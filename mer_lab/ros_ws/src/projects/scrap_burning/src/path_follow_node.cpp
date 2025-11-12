#include <ros/ros.h>

#include "path_follow_class.cpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_follow_node");
  ros::NodeHandle nh;

  PathFollowClass pfc(nh);
  pfc.attempt_follow();

  return 0;
}
