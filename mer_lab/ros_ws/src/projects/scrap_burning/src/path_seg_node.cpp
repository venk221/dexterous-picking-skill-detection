#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 

#include <stdlib.h>
#include <string>

#include "ros_pcl_manip/Filter.h"
#include "ros_pcl_manip/ToFile.h"

#include "scrap_burning/FilterPath.h"

// Defaults
// std::string image_topic = "/panda_camera/depth/points";
// std::string field_filter = "r";	// By default filter out the red field
// COMMAND RUN: rosrun scrap_burning path_seg_node /panda_camera/depth/points r 10
// int field_upper_limit = 255;	// Max filter
// int field_lower_limit = 10;	// Min filter

ros::ServiceClient save_client;
ros::ServiceClient pass_client;

bool path_filter(scrap_burning::FilterPath::Request& req,
		 scrap_burning::FilterPath::Response& res) {
  // Compose filter message
  ros_pcl_manip::Filter msg;
  msg.request.cloud = req.cloud;
  msg.request.field_name = req.field_name;
  msg.request.lower = req.field_lower;
  msg.request.upper = req.field_upper;
  msg.request.secondary_threshold = req.secondary_threshold;
  pass_client.call(msg);

  // Set response
  res.filtered_cloud = msg.response.filtered;
  ros_pcl_manip::ToFile to_file;
  to_file.request.cloud = res.filtered_cloud;
  to_file.request.filepath = "/tmp/path_seg_debug.pcd";
  save_client.call(to_file);

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_seg_node");
  ros::NodeHandle nh;

  // if(argc > 1)
  //   image_topic = std::string(argv[1]);
  // if(argc > 2)
  //   field_filter = std::string(argv[2]);
  // if(argc > 3)
  //   field_lower_limit = atoi(argv[3]);
  // if(argc > 4)
  //   field_upper_limit = atoi(argv[4]);

  save_client = nh.serviceClient<ros_pcl_manip::ToFile>("save_to_pcd");
  pass_client = nh.serviceClient<ros_pcl_manip::Filter>("passthrough_filter");
  ros::ServiceServer filter_server = nh.advertiseService("path_filter", path_filter);

  ros::spin();

  // sensor_msgs::PointCloud2::ConstPtr img_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(image_topic);
  // ROS_INFO_STREAM("Obtained image of size " << img_ptr->width);
  // ros_pcl_manip::ToFile save;
  // save.request.cloud = *img_ptr;
  // save.request.filepath = "/home/fadi/Documents/base.pcd";
  // save_client.call(save);

  // ROS_INFO("Saving files");
  // // Save filtered cloud
  // save.request.cloud = msg.response.filtered;
  // save.request.filepath = "/home/fadi/Documents/filtered.pcd";
  // save_client.call(save);

  // ROS_INFO("DONE");

  return 0;
}
