// System includes
#include <string>
#include <vector>
#include <iostream>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// scrap_burning includes
#include "cloud_processing.hpp"
#include "scrap_burning/pcl.hpp"
#include "scrap_burning/CurveFitting.h"
#include "scrap_burning/config/config.hpp"

// alias declarations
using scrap_burning::pcl::PointType;
using scrap_burning::pcl::PointNormalType;
using scrap_burning::pcl::CloudType;
using scrap_burning::pcl::CloudPtr;

std::vector<geometry_msgs::Point> interpolate(geometry_msgs::Point start, const geometry_msgs::Point &end, std::size_t count) {
  std::vector<geometry_msgs::Point> ret;

  ret.reserve(count);
  ret.push_back(start);

  double dX = (end.x - start.x) / count;
  double dY = (end.y - start.y) / count;
  double dZ = (end.z - start.z) / count;

  for(std::size_t i = 0; i < count; ++i) {
    start.x += dX;
    start.y += dY;
    start.z += dZ;
    ret.push_back(start);
  }

  return ret;
}

pcl::PointXYZ convertPointToPCL(const geometry_msgs::Point &pt) {
  return pcl::PointXYZ(pt.x, pt.y, pt.z);
}

void visualizePath(pcl::visualization::PCLVisualizer &vis, const std::vector<geometry_msgs::Point> &pts, std::size_t interpolationCount, double rad) {
  if(pts.size() < 2) return;

  std::size_t cnt = 0;
  for(std::size_t i = 0; i < pts.size() - 1; ++i) {
    for(const geometry_msgs::Point &pt : interpolate(pts[i], pts[i + 1], interpolationCount)) {
      std::stringstream oss;
      oss << "point_" << cnt;
      vis.addSphere(convertPointToPCL(pt), rad, 0, 255, 0, oss.str());
      ++cnt;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "skeleton_vis_node");
  ros::NodeHandle nh{};

  // We need the full cloud, leaf size, and interpolation count to fit a curve to it, check if we have the correct # of args
  if(argc != 5) {
    std::cerr << "Usage: " << argv[0] << " full_cloud skeleton_leaf_size(m) interpolation_count sphere_radius\n";
    return 1;
  }
  const std::string FULL_CLOUD_PATH(argv[1]);
  const double SKEL_LEAF_SIZE(std::stod(argv[2]));
  const std::size_t INTERPOLATION_COUNT(std::stoi(argv[3]));
  const double SPHERE_RADIUS(std::stod(argv[4]));

  // Load clouds
  CloudPtr fullCloud(new CloudType());
  if(pcl::io::loadPCDFile(FULL_CLOUD_PATH, *fullCloud) == -1) {
    std::cerr << "Failed to load cloud at " << FULL_CLOUD_PATH << '\n';
    return 2;
  }

  // Load config
  scrap_burning::ScrapBurningConfig sbc(nh);
  sbc.loadParams();

  // Filter cloud
  CloudPtr filteredCloud(filterCloud(nh, sbc, fullCloud));
  std::cout << "Filtered cloud size: " << filteredCloud->size() << '\n';

  // Fit curve
  std::vector<geometry_msgs::Point> skeletonPoints(fitCurve(nh, fullCloud, filteredCloud, SKEL_LEAF_SIZE, 10));
  std::cout << "Fit curve size: " << skeletonPoints.size() << '\n';

  // Visualize result
  pcl::visualization::PCLVisualizer vis{};
  vis.addPointCloud(fullCloud);
  visualizePath(vis, skeletonPoints, INTERPOLATION_COUNT, SPHERE_RADIUS);

  vis.spin();

  return 0;
}
