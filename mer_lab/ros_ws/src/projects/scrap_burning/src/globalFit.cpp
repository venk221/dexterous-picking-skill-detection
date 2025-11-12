#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "scrap_burning/FilterPath.h"

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
  ros::init(argc, argv, "global_fit_node");
  ros::NodeHandle nh;

  if(argc < 2) {
    std::cerr << "Please provide a filename\n";
    return 1;
  }

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 cloud2;

  if(pcl::io::loadPCDFile(std::string(argv[1]), cloud2) == -1) {
    std::cerr << "Could not load pcd file\n";
    return 2;
  }
  CloudPtr cloud;
  fromPCLPointCloud2(cloud2, *cloud);
  scrap_burning::FilterPath filter_path;
  filter_path.request.cloud = to_pc2(cloud);
  filter_path.request.field_name = "r";
  filter_path.request.field_lower = 10;
  filter_path.request.field_upper = 255;
  filter_path.request.secondary_threshold = 20;

  return 0;
}
