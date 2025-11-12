#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include "scrap_burning/FilterPath.h"
#include "scrap_burning/CurveFitting.h"

#include "ros_pcl_manip/Downsample.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudType;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtr;

// Helper method to convert data
CloudPtr from_pc2(const sensor_msgs::PointCloud2& pc2) {
  CloudPtr ret_cloud(new CloudType());
  pcl::PCLPointCloud2 temp_cloud;

  // Copy metadata
  temp_cloud.header.stamp = pc2.header.stamp.toNSec() / 1000ull;
  temp_cloud.header.seq = pc2.header.seq;
  temp_cloud.header.frame_id = pc2.header.frame_id;
  temp_cloud.height = pc2.height;
  temp_cloud.width = pc2.width;
  temp_cloud.fields.resize(pc2.fields.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = pc2.fields.begin();
  int i = 0;
  for(; it != pc2.fields.end(); ++i, ++it) {
    temp_cloud.fields[i].name = (*it).name;
    temp_cloud.fields[i].offset = (*it).offset;
    temp_cloud.fields[i].datatype = (*it).datatype;
    temp_cloud.fields[i].count = (*it).count;
  }
  temp_cloud.is_bigendian = pc2.is_bigendian;
  temp_cloud.point_step = pc2.point_step;
  temp_cloud.row_step = pc2.row_step;
  temp_cloud.is_dense = pc2.is_dense;

  // Copy data
  temp_cloud.data = pc2.data;

  // Place into cloud
  pcl::fromPCLPointCloud2(temp_cloud, (*ret_cloud));

  return ret_cloud;
}

// Helper methods taken from ros_pcl_manip
sensor_msgs::PointCloud2 to_pc2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud) {
  sensor_msgs::PointCloud2 ret;
  pcl::PCLPointCloud2 temp_cloud;

  pcl::toPCLPointCloud2((*in_cloud), temp_cloud);

  ret.header.stamp.fromNSec(temp_cloud.header.stamp  *1000ull);
  ret.header.seq = temp_cloud.header.seq;
  ret.header.frame_id = temp_cloud.header.frame_id;
  ret.height = temp_cloud.height;
  ret.width = temp_cloud.width;
  ret.fields.resize(temp_cloud.fields.size());
  std::vector<pcl::PCLPointField>::const_iterator it = temp_cloud.fields.begin();
  int i = 0;
  for(; it != temp_cloud.fields.end(); ++i, ++it) {
    ret.fields[i].name = (*it).name;
    ret.fields[i].offset = (*it).offset;
    ret.fields[i].datatype = (*it).datatype;
    ret.fields[i].count = (*it).count;
  }
  ret.is_bigendian = temp_cloud.is_bigendian;
  ret.point_step = temp_cloud.point_step;
  ret.row_step = temp_cloud.row_step;
  ret.is_dense = temp_cloud.is_dense;
  ret.data.swap(temp_cloud.data);

  return ret;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "benchmark_fit_node");
  ros::NodeHandle nh;

  if(argc < 3) {
    std::cerr << "Please provide a filename and output filename\n";
    return 1;
  }
  std::string ofn(argv[2]);

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 cloud2;

  if(pcl::io::loadPCDFile(std::string(argv[1]), cloud2) == -1) {
    std::cerr << "Could not load pcd file\n";
    return 2;
  }
  

  // Filter out the line
  ros::ServiceClient seg_client = nh.serviceClient<scrap_burning::FilterPath>("path_filter");
  scrap_burning::FilterPath filter_path;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  fromPCLPointCloud2(cloud2, *cloud);
  // Downsample
  ros_pcl_manip::Downsample ds;
  ds.request.cloud = to_pc2(cloud);
  ds.request.size = 0.01;
  auto _downsample_client = nh.serviceClient<ros_pcl_manip::Downsample>("downsample");
  _downsample_client.call(ds);
  cloud = from_pc2(ds.response.cloud);
  pcl::io::savePCDFileASCII("/tmp/downsampled.pcd", *cloud);
  filter_path.request.cloud = ds.response.cloud;
  ROS_INFO_STREAM("Full cloud size" << filter_path.request.cloud.data.size());
  filter_path.request.field_name = "r";
  filter_path.request.field_lower = 10;
  filter_path.request.field_upper = 255;
  filter_path.request.secondary_threshold = 20;
  seg_client.call(filter_path);
  ROS_INFO_STREAM("Filtered size: " << filter_path.response.filtered_cloud.data.size());

  // Now try curve fitting
  ros::ServiceClient fit_client = nh.serviceClient<scrap_burning::CurveFitting>("fit_curve");
  scrap_burning::CurveFitting curve_fitting;
  curve_fitting.request.filtered_cloud = filter_path.response.filtered_cloud;
  curve_fitting.request.full_cloud = filter_path.request.cloud;
  curve_fitting.request.control_points = std::stoi(argv[5]);
  curve_fitting.request.order = std::stoi(argv[3]);
  curve_fitting.request.smoothness = std::stoi(argv[4]);
  curve_fitting.request.radius = 1.0;
  curve_fitting.request.min_dist = 0.01;
  curve_fitting.request.k = 120;
  curve_fitting.request.no_transform = true;
  fit_client.call(curve_fitting);

  // Now that we have the points, save to file
  std::ofstream ofs(ofn);
  ROS_INFO_STREAM(curve_fitting.response.sampled_normals.size());
  for(int i = 0; i < curve_fitting.response.sampled_points.size(); ++i) {
    ofs << curve_fitting.response.sampled_points[i].x << ',' <<
      curve_fitting.response.sampled_points[i].y << ',' <<
      curve_fitting.response.sampled_points[i].z << '\n';
    ofs << curve_fitting.response.sampled_normals[i].x << ',' <<
      curve_fitting.response.sampled_normals[i].y << ',' <<
      curve_fitting.response.sampled_normals[i].z << '\n';
  }
  ofs.close();
  
  return 0;
}
