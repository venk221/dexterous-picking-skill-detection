#include <iostream>
#include <vector>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/surface/on_nurbs/fitting_curve_pdm.h>
// #include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/surface/on_nurbs/fitting_curve_pdm.h>
#include <pcl/pcl_macros.h>
// #include <pcl/surface/on_nurbs/nurbs_tools.h>
// #include <pcl/surface/on_nurbs/nurbs_data.h>
// #include <pcl/surface/on_nurbs/nurbs_solve.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros_pcl_manip/NormalEst.h>
#include <scrap_burning/Skeletonize.h>
#include <scrap_burning/CurveFitting.h>

typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
typedef CloudType::Ptr CloudPtr;

ros::ServiceClient norm_client;
ros::ServiceClient skel_client;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener;

// Helper methods to convert data
sensor_msgs::PointCloud2 to_pc2(CloudPtr in_cloud) {
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

CloudPtr from_pc2(const sensor_msgs::PointCloud2& pc2) {
  ROS_INFO("Converting PC2");
  CloudPtr ret_cloud(new CloudType());
  pcl::PCLPointCloud2 temp_cloud;

  // Copy metadata
  ROS_INFO("Copying metadata");
  temp_cloud.header.stamp = pc2.header.stamp.toNSec() / 1000ull;
  temp_cloud.header.seq = pc2.header.seq;
  temp_cloud.header.frame_id = pc2.header.frame_id;
  temp_cloud.height = pc2.height;
  temp_cloud.width = pc2.width;
  temp_cloud.fields.resize(pc2.fields.size());
  ROS_INFO_STREAM("Copying fields " << pc2.fields.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = pc2.fields.begin();
  int i = 0;
  for(; it != pc2.fields.end(); ++i, ++it) {
    temp_cloud.fields[i].name = (*it).name;
    temp_cloud.fields[i].offset = (*it).offset;
    temp_cloud.fields[i].datatype = (*it).datatype;
    temp_cloud.fields[i].count = (*it).count;
    ROS_INFO_STREAM("Copied field " << (*it));
  }
  temp_cloud.is_bigendian = pc2.is_bigendian;
  temp_cloud.point_step = pc2.point_step;
  temp_cloud.row_step = pc2.row_step;
  temp_cloud.is_dense = pc2.is_dense;

  // Copy data
  ROS_INFO("Copying data");
  temp_cloud.data = pc2.data;

  // Place into cloud
  ROS_INFO("Placing data into PCL Cloud");
  pcl::fromPCLPointCloud2(temp_cloud, (*ret_cloud));

  ROS_INFO("Returning cloud");
  return ret_cloud;
}

// void
// PointCloud2Vector2d(CloudPtr cloud, pcl::on_nurbs::vector_vec3d &data)
// {
//   for(const auto &p : *cloud)
//     {
//       if(!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
// 	data.emplace_back(p.x, p.y, p.z);
//     }
// }

// std::vector<geometry_msgs::Point> fit_curve(int order, CloudPtr filtered_cloud,
// 					    unsigned control_points, double radius,
// 					    double smoothness, double min_dist) {
//   pcl::on_nurbs::NurbsDataCurve data;
//   PointCloud2Vector2d(filtered_cloud, data.interior);
//   std::cout << "Cloud size is " << data.interior.size() << "\n";

//   // Curve params
//   std::cout << "Fitting with " << control_points << " control pts\n";

//   pcl::on_nurbs::FittingCurve::Parameter curve_params;
//   curve_params.smoothness = smoothness;

//   // Setup curve
//   ROS_INFO("Solving curve weights");
//   ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve::initNurbsCurvePCA(order, data.interior, control_points, radius);
//   std::cout << "Created curve with order " << order << "\n";
//   std::cout << curve.IsClosed() << std::endl;
//   pcl::on_nurbs::FittingCurve fit(&data, curve);
//   fit.refine();
//   fit.refine();
//   fit.refine();
//   fit.assemble(curve_params);
//   fit.solve();

//   // Discretize and return data
//   std::vector<geometry_msgs::Point> ret;
//   ROS_INFO("Discretizing data");
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::on_nurbs::Triangulation::convertCurve2PointCloud(fit.m_nurbs, cloud_temp, 8);
//   std::vector<pcl::PointXYZRGB> start_line_pts;
//   std::vector<pcl::PointXYZRGB> end_line_pts;
//   for(std::size_t i = 0; i < cloud_temp->size() - 1; i++) {
//     pcl::PointXYZRGB &p1 = cloud_temp->at(i);
//     pcl::PointXYZRGB &p2 = cloud_temp->at(i + 1);
//     double dx, dy, dz;
//     for(int i = 0; i < data.interior.size(); ++i) {
//       dx = data.interior[i][0] - p1.x;
//       dy = data.interior[i][1] - p1.y;
//       dz = data.interior[i][2] - p1.z;
//       if(dx*dx+dy*dy+dz*dz <= min_dist*min_dist) {
// 	geometry_msgs::Point sampled_point;
// 	sampled_point.x = p1.x;
// 	sampled_point.y = p1.y;
// 	sampled_point.z = p1.z;
// 	ret.push_back(sampled_point);
// 	start_line_pts.push_back(p1);
// 	end_line_pts.push_back(p2);
// 	break;
//       }
//     }
//     // viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
//   }

//   return ret;
// }

constexpr int MAX_ATTEMPTS = 20;

std::vector<geometry_msgs::Point> getNormals(CloudPtr cloud, const std::vector<geometry_msgs::Point>& pts, int k) {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // Set the full cloud to be used in nearest neighbors
  kdtree.setInputCloud(cloud);

  // Estimate normals, first by finding the closest point
  ROS_INFO("Estimating Normals");
  ros_pcl_manip::NormalEst norm_est;
  // We need to use the full cloud when estimating normals
  norm_est.request.in_cloud = to_pc2(cloud);
  norm_est.request.k = k;
  std::vector<int> index(1);
  std::vector<float> dist(1);
  for(int i = 0; i < pts.size(); ++i) { // For every point, find the closest in the cloud
    kdtree.nearestKSearch(pcl::PointXYZ(pts[i].x,
					pts[i].y,
					pts[i].z),
    			  1, index, dist);
    norm_est.request.pts.push_back(index[0]);
  }
  norm_client.call(norm_est);

  return norm_est.response.normals;
}

std::vector<geometry_msgs::Point> fit_skeleton(CloudPtr filtered_cloud, double leaf_size, bool view) {
  scrap_burning::Skeletonize skel;
  
  skel.request.cloud = to_pc2(filtered_cloud);
  skel.request.leaf_size = leaf_size;
  skel.request.view = view;
  skel_client.call(skel);

  return skel.response.skeleton;
}

// Service callback
bool fit_curve(scrap_burning::CurveFitting::Request& req,
	       scrap_burning::CurveFitting::Response& res) {
  ROS_INFO("Received curve fit request");
  // Transform cloud
  sensor_msgs::PointCloud2 tf_filtered_cloud;
  sensor_msgs::PointCloud2 tf_full_cloud;
  int attempts = 0;
  if(req.no_transform) {
    tf_filtered_cloud = req.filtered_cloud;
    tf_full_cloud = req.full_cloud;
  }
  while(!req.no_transform) {
    try {
      ROS_INFO("Attempting to transform frames");
      geometry_msgs::TransformStamped tfStamped =
  	tfBuffer.lookupTransform("panda_link0", req.full_cloud.header.frame_id, ros::Time(0));
      tf2::doTransform(req.filtered_cloud, tf_filtered_cloud, tfStamped);
      tf2::doTransform(req.full_cloud, tf_full_cloud, tfStamped);
      break;
    } catch(tf2::TransformException& ex) {
      attempts++;
      if(attempts >= MAX_ATTEMPTS) {
	ROS_WARN("Too many attempts, moving on without transformation");
	tf_filtered_cloud = req.filtered_cloud;
	tf_full_cloud = req.full_cloud;
	break;
      }
      ROS_WARN_STREAM(ex.what());
      ROS_WARN("Failed to transform to world frame, trying again...");
      ros::Duration d(1.0);
      d.sleep();
    }
  }

  ROS_INFO("Done transforming, attempting to fit curve");

  CloudPtr filtered_cloud = from_pc2(tf_filtered_cloud);
  CloudPtr full_cloud = from_pc2(tf_full_cloud);
  ROS_INFO_STREAM("Got cloud with size " << filtered_cloud->size());

  // Based on the use_skeleton, we either fit a curve or a skeleton
  if(req.use_skeleton) {
    ROS_INFO("Using skeletonization to fit curve");
    res.sampled_points = fit_skeleton(filtered_cloud, req.skeleton_leaf_size, !req.hide_viewer);
  }
  else {
    ROS_ERROR("Fitting PCL Curves is no longer supported");
    // ROS_INFO("Fitting PCL Curve to data");
    // res.sampled_points = fit_curve(req.order, filtered_cloud,
    // 				   req.control_points, req.radius,
    // 				   req.smoothness, req.min_dist);
  }
  
  res.sampled_normals = getNormals(full_cloud, res.sampled_points, req.k);

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "curve_fitting_node");
  ros::NodeHandle nh;

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  // Setup services
  ros::ServiceServer fit_server = nh.advertiseService("fit_curve", fit_curve);
  skel_client = nh.serviceClient<scrap_burning::Skeletonize>("skeletonize");
  norm_client = nh.serviceClient<ros_pcl_manip::NormalEst>("normal_estimation");

  // Spin
  ros::spin();
  
  return 0;
}
