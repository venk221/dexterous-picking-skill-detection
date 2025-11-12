#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include "ros_pcl_manip/ros_pcl_manip.h"

ROS_PCL::ROS_PCL(ros::NodeHandle& nh) : _nh(nh) {_setup_services();}
ROS_PCL::~ROS_PCL() {}

CloudPtr ROS_PCL::downsample(CloudPtr in_cloud, float leaf_size) {
  if(leaf_size == 0.0f)		// No change in size
    return in_cloud;
  CloudPtr ret(new CloudType());
  pcl::VoxelGrid<PointType> vg;

  vg.setInputCloud(in_cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*ret);

  return ret;
}
CloudPtr ROS_PCL::seg_plane(CloudPtr in_cloud, bool remove, float leaf_size) {
  // Variables
  CloudPtr downsampledCloud = downsample(in_cloud, leaf_size);
  CloudPtr extractedPlane(new CloudType());
  pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
  pcl::ExtractIndices<PointType> extract;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointType> planeSeg;

  // Setup segmentation
  planeSeg.setOptimizeCoefficients(true);
  planeSeg.setModelType(pcl::SACMODEL_PLANE);
  planeSeg.setMethodType(pcl::SAC_RANSAC);
  planeSeg.setDistanceThreshold(0.01);
  planeSeg.setInputCloud(downsampledCloud);

  // Segment
  planeSeg.segment(*planeInliers, *coefficients);

  // Extract indices
  extract.setInputCloud(downsampledCloud);
  extract.setIndices(planeInliers);
  extract.setNegative(false);
  extract.filter(*extractedPlane);
  if(remove) {			// Replace the current cloud with no plane
    extract.setNegative(true);
    extract.filter(*in_cloud);
  }

  return extractedPlane;
}
double ROS_PCL::getResolution(CloudConstPtr cloud) {
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i) {
    if (!std::isfinite ((*cloud)[i].x))
      continue;
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2) {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
    res /= n_points;
  return res;
}
// Check if res_invariant flag has been set first
CorrespondenceParams ROS_PCL::scaleInvariant(const CorrespondenceParams& p, double modelRes) {
  float res = static_cast<float>(modelRes);
  CorrespondenceParams ret;
  if(modelRes != 0.0f) {
    ret.model_ss = p.model_ss * res;
    ret.scene_ss = p.scene_ss * res;
    ret.rf_rad = p.rf_rad * res;
    ret.descr_rad = p.descr_rad * res;
    ret.cg_size = p.cg_size * res;
  }

  return ret;
}
NormalCloudPtr ROS_PCL::computeNormals(CloudConstPtr in_cloud, int k, geometry_msgs::Point view) {
  ROS_INFO("Computing normals");
  pcl::NormalEstimationOMP<PointType, NormalType> norm;
  NormalCloudPtr ret(new NormalCloud);

  norm.setViewPoint(view.x, view.y, view.z);
  norm.setKSearch(k);
  // norm.setRadiusSearch(0.03);
  norm.setInputCloud(in_cloud);
  norm.compute(*ret);

  return ret;
}
CloudPtr ROS_PCL::extractKeypoints(CloudPtr in_cloud, float search) {
  ROS_INFO_STREAM("Extracting keypoints with search " << search);
  CloudPtr ret(new CloudType());
  pcl::UniformSampling<PointType> us;

  us.setInputCloud(in_cloud);
  us.setRadiusSearch(search);

  us.filter(*ret);

  return ret;
}
DescriptorCloudPtr ROS_PCL::computeDescriptors(CloudPtr in_cloud, CloudPtr in_keypoints, NormalCloudPtr in_normal, float search) {
  ROS_INFO_STREAM("Computing descriptors with search " << search);
  DescriptorCloudPtr ret(new DescriptorCloud());
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;

  descr_est.setRadiusSearch(search);
  descr_est.setInputCloud(in_keypoints);
  descr_est.setInputNormals(in_normal);
  descr_est.setSearchSurface(in_cloud);

  descr_est.compute(*ret);

  return ret;
}
pcl::CorrespondencesPtr ROS_PCL::getCorrespondences(DescriptorCloudPtr model_descs,
						    DescriptorCloudPtr scene_descs) {
  ROS_INFO_STREAM("Getting correspondences");
  pcl::CorrespondencesPtr ret(new pcl::Correspondences());
  pcl::KdTreeFLANN<DescriptorType> match_search;

  match_search.setInputCloud(model_descs);

  for(std::size_t i = 0; i < scene_descs->size(); ++i) {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    if(!std::isfinite(scene_descs->at(i).descriptor[0]))
      continue;
    int found_neighs = match_search.nearestKSearch(scene_descs->at(i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
      ret->push_back(corr);
    }
  }

  return ret;
}
std::vector<geometry_msgs::Pose> ROS_PCL::geometricConsistency(CloudPtr model_keypts, CloudPtr scene_keypts,
							       pcl::CorrespondencesPtr msc,
							       float size, float thresh) {
  std::vector<geometry_msgs::Pose> ret;
  pcl::GeometricConsistencyGrouping<PointType, PointType> gc;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  gc.setGCSize(size);
  gc.setGCThreshold(thresh);
  gc.setInputCloud(model_keypts);
  gc.setSceneCloud(scene_keypts);
  gc.setModelSceneCorrespondences(msc);

  gc.recognize(rototranslations, clustered_corrs);

  // Conversion
  return ret;
}

std::vector<geometry_msgs::Pose> ROS_PCL::correspondenceGrouping(CloudPtr scene, CloudPtr model,
								 bool cloud_res, CorrespondenceParams params) {
  std::vector<geometry_msgs::Pose> ret;

  // Set invariant
  if(cloud_res)
    params = scaleInvariant(params, getResolution(model));

  // Compute normals
  NormalCloudPtr modelNormals = computeNormals(model);
  NormalCloudPtr sceneNormals = computeNormals(scene);

  // Extract keypoints
  CloudPtr modelKeypts = extractKeypoints(model, params.model_ss);
  CloudPtr sceneKeypts = extractKeypoints(scene, params.scene_ss);

  // Compute descritors
  DescriptorCloudPtr modelDesc = computeDescriptors(model, modelKeypts, modelNormals, params.descr_rad);
  DescriptorCloudPtr sceneDesc = computeDescriptors(scene, sceneKeypts, sceneNormals, params.descr_rad);

  // Compute correspondences
  pcl::CorrespondencesPtr modelCorr = getCorrespondences(modelDesc, sceneDesc);

  // // Cluster
  ret = geometricConsistency(modelKeypts, sceneKeypts, modelCorr, params.cg_size, params.cg_thresh);

  return ret;
}
CloudPtr ROS_PCL::filter_cloud(CloudPtr in_cloud, std::string field_name,
			       float lower, float upper, float secondaryThreshold) {
  CloudPtr filtered_cloud(new CloudType());

  // Construct filters
  pcl::ConditionalRemoval<PointType> filter;
  pcl::ConditionAnd<PointType>::Ptr cond(new pcl::ConditionAnd<PointType>());
  if(field_name == "r" || field_name == "g" || field_name == "b") { // Color field
    ROS_INFO("Detected color field");
    ROS_INFO_STREAM("Comparing " << field_name << " between " << lower << " and " << upper);
    pcl::PackedRGBComparison<PointType>::Ptr condl(new pcl::PackedRGBComparison<PointType>(field_name,
											   pcl::ComparisonOps::GT,
											   lower));
    pcl::PackedRGBComparison<PointType>::Ptr condu(new pcl::PackedRGBComparison<PointType>(field_name,
											   pcl::ComparisonOps::LT,
											   upper));
    pcl::PackedRGBComparison<PointType>::Ptr nog(new pcl::PackedRGBComparison<PointType>("g",
											 pcl::ComparisonOps::LT,
											 secondaryThreshold));
    pcl::PackedRGBComparison<PointType>::Ptr nob(new pcl::PackedRGBComparison<PointType>("b",
											 pcl::ComparisonOps::LT,
											 secondaryThreshold));
    pcl::PackedRGBComparison<PointType>::Ptr nor(new pcl::PackedRGBComparison<PointType>("r",
											 pcl::ComparisonOps::LT,
											 secondaryThreshold));
    cond->addComparison(condl);
    cond->addComparison(condu);
    if(field_name == "r" || field_name == "b")
      cond->addComparison(nog);
    if(field_name == "r" || field_name == "g")
      cond->addComparison(nob);
    if(field_name == "g" || field_name == "b")
      cond->addComparison(nor);
  }
  else {			// Non-RGB, not supported yet
  }

  filter.setInputCloud(in_cloud);
  filter.setCondition(cond);

  filter.filter(*filtered_cloud);

  return filtered_cloud;
}

bool ROS_PCL::downsample_service(ros_pcl_manip::Downsample::Request& req,
				 ros_pcl_manip::Downsample::Response& res) {
  ROS_INFO("Received downsample_service");
  CloudPtr tempPtr = from_pc2(req.cloud);
  tempPtr = downsample(tempPtr, req.size);

  res.cloud = to_pc2(tempPtr);

  return true;
}
bool ROS_PCL::segment_plane_service(ros_pcl_manip::SegmentPlane::Request& req,
				    ros_pcl_manip::SegmentPlane::Response& res) {
  ROS_INFO("Received segment_plane_service");
  CloudPtr negativePtr = from_pc2(req.cloud);
  CloudPtr planePtr= seg_plane(negativePtr, true, req.leaf_size);

  res.plane = to_pc2(planePtr);
  res.negativeCloud = to_pc2(negativePtr);

  return true;
}
bool ROS_PCL::cor_group_service(ros_pcl_manip::CorrGroup::Request& req,
				ros_pcl_manip::CorrGroup::Response& res) {
  ROS_INFO("Received cor_group_service");
  // Conversions
  CloudPtr modelPtr = from_pc2(req.model);
  CloudPtr scenePtr = from_pc2(req.scene);

  CorrespondenceParams p;

  std::vector<geometry_msgs::Pose> ret = correspondenceGrouping(modelPtr, scenePtr, req.invariant, p);

  res.detectedModels = ret;
  return true;
}
bool ROS_PCL::filter_service(ros_pcl_manip::Filter::Request& req,
			     ros_pcl_manip::Filter::Response& res) {
  ROS_INFO("Received filter service");

  CloudPtr cloud = from_pc2(req.cloud);
  CloudPtr ret = filter_cloud(cloud, req.field_name, req.lower, req.upper, req.secondary_threshold);

  res.filtered = to_pc2(ret);

  return true;
}
bool ROS_PCL::load_service(ros_pcl_manip::LoadFile::Request& req,
			   ros_pcl_manip::LoadFile::Response& res) {
  ROS_INFO("Received load_service");
  CloudPtr fileCloud(new CloudType());

  if(pcl::io::loadPCDFile<PointType>(req.filepath, *fileCloud) == -1) {
    ROS_ERROR_STREAM("Could not load file " << req.filepath);
    return false;
  }

  res.cloud = to_pc2(fileCloud);
  res.cloud.header.frame_id = "world"; // If no frame given, set to world
  if(req.frame != "")
    res.cloud.header.frame_id = req.frame;
  return true;
}
bool ROS_PCL::save_service(ros_pcl_manip::ToFile::Request& req,
			   ros_pcl_manip::ToFile::Response& res) {
  ROS_INFO("Received save_service");
  CloudPtr fileCloud = from_pc2(req.cloud);

  if(pcl::io::savePCDFileASCII(req.filepath, *fileCloud) == -1) {
    ROS_ERROR_STREAM("Could not save cloud to file " << req.filepath);
    return false;
  }

  return true;
}
bool ROS_PCL::norm_service(ros_pcl_manip::NormalEst::Request& req,
			   ros_pcl_manip::NormalEst::Response& res) {
  ROS_INFO("Received norm_service");

  CloudPtr reqCloud = from_pc2(req.in_cloud);
  NormalCloudPtr normCloud = computeNormals(reqCloud, req.k, req.view);
  for(int i = 0; i < req.pts.size(); ++i) {
    geometry_msgs::Point norm;
    norm.x = normCloud->points[req.pts[i]].normal_x;
    norm.y = normCloud->points[req.pts[i]].normal_y;
    norm.z = normCloud->points[req.pts[i]].normal_z;
    // norm.x = normCloud->points[i].normal_x + reqCloud->points[i].x;
    // norm.y = normCloud->points[i].normal_y + reqCloud->points[i].y;
    // norm.z = normCloud->points[i].normal_z + reqCloud->points[i].z;
    res.normals.push_back(norm);
  }

  return true;
}

bool ROS_PCL::outlier_removal_service(ros_pcl_manip::OutlierRemoval::Request& req,
				      ros_pcl_manip::OutlierRemoval::Response& res) {
  ROS_INFO("Received outlier_removal_service");

  CloudPtr reqCloud = from_pc2(req.cloud);
  CloudPtr filtered(new CloudType);
  
  StatisticalOutlierRemovalType sor;
  sor.setInputCloud(reqCloud);
  sor.setMeanK(req.mean_k);
  sor.setStddevMulThresh(req.stddev_thresh);
  sor.filter(*filtered);

  res.filtered_cloud = to_pc2(filtered);

  return true;
}

bool ROS_PCL::voxelize_service(ros_pcl_manip::Voxelize::Request& req,
			       ros_pcl_manip::Voxelize::Response& res) {
  ROS_INFO("Received voxelize_service");

  CloudPtr reqCloud = from_pc2(req.cloud);
  OctreeType octree(req.res);
  if(req.custom_bbx) {
    ROS_INFO("Setting up custom bounding box");
    octree.defineBoundingBox(req.bbxMinPt.x, req.bbxMinPt.y, req.bbxMinPt.z,
			     req.bbxMaxPt.y, req.bbxMaxPt.y, req.bbxMaxPt.z);
  }
  octree.setInputCloud(reqCloud);
  octree.addPointsFromInputCloud();
  ROS_INFO("Beginning getOccupiedVoxelCenters");
  std::vector<PointType, Eigen::aligned_allocator<PointType>> occVoxels;
  octree.getOccupiedVoxelCenters(occVoxels);
  ROS_INFO("Getting bounding box");
  double xmin, ymin, zmin;
  double xmax, ymax, zmax;
  octree.getBoundingBox(xmin, ymin, zmin,
			xmax, ymax, zmax);
  ROS_INFO("Inserting data");
  res.x.push_back(xmin);
  res.y.push_back(ymin);
  res.z.push_back(zmin);
  res.x.push_back(xmax);
  res.y.push_back(ymax);
  res.z.push_back(zmax);

  ROS_INFO("Populating data");
  for(const auto& pt : occVoxels) {
    res.x.push_back(pt.x);
    res.y.push_back(pt.y);
    res.z.push_back(pt.z);
  }
  ROS_INFO("DONE");
  return true;
}

CloudPtr ROS_PCL::from_pc2(const sensor_msgs::PointCloud2& pc2) {
  ROS_INFO("Converting PC2");
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
sensor_msgs::PointCloud2 ROS_PCL::to_pc2(CloudPtr in_cloud) {
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

void ROS_PCL::_setup_services() {
  _down_server = _nh.advertiseService("downsample",
				      &ROS_PCL::downsample_service,
				      this);
  _plane_server = _nh.advertiseService("segment_plane",
				       &ROS_PCL::segment_plane_service,
				       this);
  _cor_server = _nh.advertiseService("correspondence_grouping",
				     &ROS_PCL::cor_group_service,
				     this);
  _load_server = _nh.advertiseService("load_pcd_file",
				      &ROS_PCL::load_service,
				      this);
  _save_server = _nh.advertiseService("save_to_pcd",
				      &ROS_PCL::save_service,
				      this);
  _pass_server = _nh.advertiseService("passthrough_filter",
				      &ROS_PCL::filter_service,
				      this);
  _norm_server = _nh.advertiseService("normal_estimation",
				      &ROS_PCL::norm_service,
				      this);
  _outlier_removal_server = _nh.advertiseService("outlier_removal",
						 &ROS_PCL::outlier_removal_service,
						 this);
  _voxelize_server = _nh.advertiseService("voxelize",
					  &ROS_PCL::voxelize_service,
					  this);
}
