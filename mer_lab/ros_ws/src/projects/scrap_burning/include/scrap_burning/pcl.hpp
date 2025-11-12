#pragma once

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

// STL includes
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace scrap_burning {
  namespace pcl {

    typedef ::pcl::PointXYZRGB PointType;
    typedef ::pcl::PointXYZRGBNormal PointNormalType;
    typedef ::pcl::PointCloud<PointType> CloudType;
    typedef CloudType::Ptr CloudPtr;

    inline CloudPtr from_pc2(const sensor_msgs::PointCloud2& pc2) {
      CloudPtr ret_cloud(new CloudType());
      ::pcl::PCLPointCloud2 temp_cloud;

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
      ::pcl::fromPCLPointCloud2(temp_cloud, (*ret_cloud));

      return ret_cloud;
    }
    inline sensor_msgs::PointCloud2 to_pc2(CloudPtr in_cloud) {
      sensor_msgs::PointCloud2 ret;
      ::pcl::PCLPointCloud2 temp_cloud;

      ::pcl::toPCLPointCloud2((*in_cloud), temp_cloud);

      ret.header.stamp.fromNSec(temp_cloud.header.stamp  *1000ull);
      ret.header.seq = temp_cloud.header.seq;
      ret.header.frame_id = temp_cloud.header.frame_id;
      ret.height = temp_cloud.height;
      ret.width = temp_cloud.width;
      ret.fields.resize(temp_cloud.fields.size());
      std::vector<::pcl::PCLPointField>::const_iterator it = temp_cloud.fields.begin();
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

  } // namespace pcl
}   // namespace scrap_burning
