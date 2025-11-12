#ifndef CLOUD_PROCESSING_HPP
#define CLOUD_PROCESSING_HPP

#include <ros/ros.h>
#include <unordered_set>

#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

#include "scrap_burning/pcl.hpp"
#include "ros_pcl_manip/Voxelize.h"
#include "scrap_burning/FilterPath.h"
#include "scrap_burning/CurveFitting.h"
#include "scrap_burning/config/config.hpp"

using scrap_burning::pcl::PointType;
using scrap_burning::pcl::PointNormalType;
using scrap_burning::pcl::CloudType;
using scrap_burning::pcl::CloudPtr;

std::vector<float> getNearestCorrespondences(CloudPtr measured, CloudPtr groundTruth) {
  std::vector<float> ret;
  pcl::KdTreeFLANN<PointType> tree;

  ret.reserve(measured->size());
  tree.setInputCloud(groundTruth);

  std::vector<int> nearestIndex(1);
  std::vector<float> nearestDist(1);
  for(const auto &pt : *measured) {
    int found = tree.nearestKSearch(pt, 1, nearestIndex, nearestDist);
    if(found > 0)		// Found a neighbour
      ret.push_back(nearestDist[0]);
  }

  return ret;
}

pcl::PolygonMesh reconstructPoisson(CloudPtr cloud, int normalK) {
  pcl::PolygonMesh ret;
  pcl::Poisson<PointNormalType> poisson;
  pcl::PointCloud<PointNormalType>::Ptr normalCloud(new pcl::PointCloud<PointNormalType>());

  // Compute normals
  pcl::NormalEstimationOMP<PointType, PointNormalType> normalEst;
  normalEst.setKSearch(normalK);
  normalEst.setInputCloud(cloud);
  normalEst.compute(*normalCloud);

  poisson.setInputCloud(normalCloud);
  poisson.performReconstruction(ret);

  return ret;
}

CloudPtr removeOutliers(CloudPtr cloud, int minPts) {
  CloudPtr ret(new CloudType());
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointType> euclideanExtractor;

  euclideanExtractor.setInputCloud(cloud);
  euclideanExtractor.setSearchMethod(tree);
  euclideanExtractor.setMinClusterSize(minPts);
  euclideanExtractor.setMaxClusterSize(5000000);
  euclideanExtractor.setClusterTolerance(0.005);
  euclideanExtractor.extract(clusterIndices);

  // Reserve enough space
  std::size_t totalSize = 0;
  for(const auto &indices : clusterIndices)
    totalSize += indices.indices.size();
  ret->reserve(totalSize);
  // Insert points
  for(const auto &indices : clusterIndices)
    for(const auto &index : indices.indices)
      ret->push_back(cloud->operator[](index));

  return ret;
}

CloudPtr filterCloudOutsideBounds(CloudPtr cloud, Eigen::Vector3f minBounds, Eigen::Vector3f maxBounds) {
  CloudPtr ret(new CloudType());

  for(const auto &pt : *cloud) {
    if(pt.x >= minBounds[0] && pt.x <= maxBounds[0] &&
       pt.y >= minBounds[1] && pt.y <= maxBounds[1] &&
       pt.z >= minBounds[2] && pt.z <= maxBounds[2]) {
      ret->push_back(pt);
    }
  }

  return ret;
}

std::vector<geometry_msgs::Point> fitCurve(ros::NodeHandle &nh, CloudPtr fullCloud, CloudPtr filteredCloud, double leafSize, double k) {
  ros::ServiceClient filtClient(nh.serviceClient<scrap_burning::CurveFitting>("fit_curve"));

  // Setup message parameters
  scrap_burning::CurveFitting curveFit{};
  curveFit.request.full_cloud = scrap_burning::pcl::to_pc2(fullCloud);
  curveFit.request.filtered_cloud = scrap_burning::pcl::to_pc2(filteredCloud);
  // curveFit.request.skeleton_leaf_size = leafSize;
  // curveFit.request.use_skeleton = true;
  curveFit.request.use_skeleton = false;
  curveFit.request.control_points = 40;
  curveFit.request.order = 3;
  curveFit.request.min_dist = 1.0;
  curveFit.request.smoothness = 0.01;
  curveFit.request.no_transform = true;
  curveFit.request.hide_viewer = true;
  curveFit.request.k = k;

  // Call service
  filtClient.call(curveFit);

  // Return points along the curve
  return curveFit.response.sampled_points;
}

CloudPtr filterCloud(ros::NodeHandle &nh, const scrap_burning::ScrapBurningConfig &sbc, CloudPtr cloud) {
  ros::ServiceClient segClient = nh.serviceClient<scrap_burning::FilterPath>("path_filter");

  scrap_burning::FilterPath fp;
  fp.request.cloud = scrap_burning::pcl::to_pc2(cloud);
  fp.request.field_name = sbc.filter_field;
  fp.request.field_lower = sbc.filter_lower;
  fp.request.field_upper = sbc.filter_upper;
  fp.request.secondary_threshold = sbc.filter_secondary_thresh;
  segClient.call(fp);

  return scrap_burning::pcl::from_pc2(fp.response.filtered_cloud);
}

CloudPtr downsampleCloud(ros::NodeHandle &nh, double leafSize, CloudPtr cloud) {
  CloudPtr ret(new CloudType());

  ros::ServiceClient voxClient = nh.serviceClient<ros_pcl_manip::Voxelize>("voxelize");

  ros_pcl_manip::Voxelize vox;
  vox.request.cloud = scrap_burning::pcl::to_pc2(cloud);
  vox.request.res = leafSize;
  vox.request.custom_bbx = true;
  vox.request.bbxMinPt.x = -3.0;
  vox.request.bbxMinPt.y =  vox.request.bbxMinPt.x;
  vox.request.bbxMinPt.z =  vox.request.bbxMinPt.x;
  vox.request.bbxMaxPt.x = -vox.request.bbxMinPt.x;
  vox.request.bbxMaxPt.y = -vox.request.bbxMinPt.x;
  vox.request.bbxMaxPt.z = -vox.request.bbxMinPt.x;
  voxClient.call(vox);

  // Convert array of x, y, z to pointcloud
  // First two points are the bounding box min + max
  ret->reserve(vox.response.x.size() - 2);
  for(std::size_t i = 2; i < vox.response.x.size(); ++i) {
    PointType pt(static_cast<uint8_t>(255), 255, 255);
    pt.x = vox.response.x[i];
    pt.y = vox.response.y[i];
    pt.z = vox.response.z[i];
    ret->push_back(pt);
  }

  return ret;
}

// Returns indices in the ground truth cloud that were found in the measured to within margin
pcl::PointIndices::Ptr computeMatches(CloudPtr groundTruth, CloudPtr measured, double margin) {
  // Store unique indices here (we should not have overlap)
  std::unordered_set<std::size_t> uniqueIndices;

  // Once done, place them in here
  pcl::PointIndices::Ptr ret(new pcl::PointIndices());

  pcl::KdTreeFLANN<PointType> tree;
  tree.setInputCloud(measured);

  std::vector<int> nearestIndices(measured->size());
  std::vector<float> nearestDists(nearestIndices.size());
  for(std::size_t i = 0; i < groundTruth->size(); ++i) {
    std::size_t searchCount = tree.radiusSearch(groundTruth->operator[](i), margin, nearestIndices, nearestDists);
    uniqueIndices.insert(nearestIndices.begin(), nearestIndices.begin() + searchCount);
  }

  ret->indices.insert(ret->indices.end(), uniqueIndices.begin(), uniqueIndices.end());

  return ret;
}

CloudPtr extractIndices(CloudPtr cloud, pcl::PointIndices::Ptr indices) {
  CloudPtr ret(new CloudType());

  // Setup index extractor
  pcl::ExtractIndices<PointType> extractIndices;
  extractIndices.setInputCloud(cloud);
  extractIndices.setIndices(indices);

  // Extract
  extractIndices.filter(*ret);

  return ret;
}

void colorCloud(CloudPtr cloud, uint8_t r, uint8_t g, uint8_t b, pcl::PointIndices::Ptr indices=nullptr) {
  if(indices == nullptr) {
    for(auto &pt : *cloud) {
      pt.r = r;
      pt.g = g;
      pt.b = b;
    }
  }
  else {
    for(auto ptIndex : indices->indices) {
      PointType &pt = cloud->operator[](ptIndex);
      pt.r = r;
      pt.g = g;
      pt.b = b;
    }
  }
}

#endif
