#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "scrap_burning/FilterPath.h"
#include "scrap_burning/CurveFitting.h"

#include "moveit_planner/MoveAway.h"
#include "moveit_planner/MoveJoint.h"

#include "ros_pcl_manip/ToFile.h"
#include "ros_pcl_manip/Downsample.h"

#include "scrap_burning/PCLConcatRequest.h"

#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <Eigen/Dense>

sensor_msgs::PointCloud2 _img;
ros::ServiceClient seg_client;
ros::ServiceClient fit_client;

void imgCallback(const sensor_msgs::PointCloud2::ConstPtr& img) {
  _img = (*img);
}

// Returns the squared distance between two points p1 & p2
double get_dist(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return (p1.x-p2.x)*(p1.x-p2.x) +
    (p1.y-p2.y)*(p1.y-p2.y) +
    (p1.z-p2.z)*(p1.z-p2.z);
}

geometry_msgs::Pose toTargetPose(const geometry_msgs::Point& pos, const geometry_msgs::Point& norm) {
  geometry_msgs::Pose ret;
  // Position is easy, the orientation is difficult
  ret.position = pos;

  Eigen::Matrix3d trans(3, 3);
  // Rotation - z
  trans(0, 2) = norm.x;
  trans(1, 2) = norm.y;
  trans(2, 2) = norm.z;
  // trans(0, 2) = 0.0; trans(1, 2) = 0.0; trans(2, 2) = -1.0;
  // Rotation - y
  trans(2, 1) = 0;		// Keep this horizontal
  trans(0, 1) = trans(1, 2);
  trans(1, 1) = -trans(0, 2);
  // Rotation - x
  trans(0, 0) = trans(1, 1) * trans(2, 2) - trans(1, 2) * trans(2, 1);
  trans(1, 0) = trans(0, 2) * trans(2, 1) - trans(0, 1) * trans(2, 2);
  trans(2, 0) = trans(0, 1) * trans(1, 2) - trans(0, 2) * trans(1, 1);
  // Convert to quaternion
  Eigen::Quaterniond q(trans);
  q = q.normalized();
  ret.orientation.x = q.x();
  ret.orientation.y = q.y();
  ret.orientation.z = q.z();
  ret.orientation.w = q.w();

  return ret;
}

visualization_msgs::Marker setup_marker(int id, double scale=0.01) {
  visualization_msgs::Marker ret;
  // Scale
  ret.scale.x = scale;
  ret.scale.y = scale;
  ret.scale.z = scale;
  // Color
  ret.color.a = 1.0;
  // Orientation
  ret.pose.orientation.w = 1.0;
  // ID
  ret.id = id;
  // Header
  ret.header.frame_id = "world";
  ret.header.stamp = ros::Time();

  return ret;
}

struct Line {
  geometry_msgs::Point a;
  geometry_msgs::Point b;
};

double dotProd(const Line& l1, const Line& l2) {
  geometry_msgs::Point v1;
  v1.x = l1.b.x - l1.a.x;
  v1.y = l1.b.y - l1.a.y;
  v1.z = l1.b.z - l1.a.z;
  geometry_msgs::Point v2;
  v2.x = l2.b.x - l2.a.x;
  v2.y = l2.b.y - l2.a.y;
  v2.z = l2.b.z - l2.a.z;

  return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

void recv_img() {
  _img = sensor_msgs::PointCloud2();
  while(_img.data.size() == 0)
    ros::spinOnce();
}

inline scrap_burning::CurveFitting filter_fit() {
  // Filter
  scrap_burning::FilterPath filter_path;
  filter_path.request.cloud = _img;
  filter_path.request.field_name = "r";
  filter_path.request.field_lower = 10;
  filter_path.request.field_upper = 255;
  seg_client.call(filter_path);
  // Fit
  scrap_burning::CurveFitting curve_fitting;
  curve_fitting.request.filtered_cloud = filter_path.response.filtered_cloud;
  curve_fitting.request.control_points = 20; // Original: 20
  curve_fitting.request.order = 3;
  curve_fitting.request.smoothness = 100.0;
  curve_fitting.request.radius = 1.0;
  curve_fitting.request.min_dist = 0.01;
  curve_fitting.request.hide_viewer = true;
  fit_client.call(curve_fitting);

  return curve_fitting;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_follower_node");
  ros::NodeHandle nh;

  ros::ServiceClient seg_client = nh.serviceClient<scrap_burning::FilterPath>("path_filter");
  ros::ServiceClient fit_client = nh.serviceClient<scrap_burning::CurveFitting>("fit_curve");
  ros::ServiceClient concat_req_client = nh.serviceClient<scrap_burning::PCLConcatRequest>("concat_server");
  ros::ServiceClient move_joint_client = nh.serviceClient<moveit_planner::MoveJoint>("move_to_joint_space");
  ros::ServiceClient move_away_client = nh.serviceClient<moveit_planner::MoveAway>("move_away_point");
  ros::ServiceClient save_img_client = nh.serviceClient<ros_pcl_manip::ToFile>("save_to_pcd");
  ros::ServiceClient downsample_client = nh.serviceClient<ros_pcl_manip::Downsample>("downsample");

  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  visualization_msgs::Marker target_marker = setup_marker(2);
  target_marker.type = visualization_msgs::Marker::SPHERE;
  target_marker.color.b = 1.0;
  // target_marker.header.frame_id = "world";
  // target_marker.header.stamp = ros::Time();
  // target_marker.id = 2;
  // target_marker.type = visualization_msgs::Marker::SPHERE;
  // target_marker.color.a = 1.0;
  // target_marker.color.b = 1.0;
  // target_marker.scale.x = 0.01;
  // target_marker.scale.y = 0.01;
  // target_marker.scale.z = 0.01;
  // target_marker.pose.orientation.w = 1.0;
  visualization_msgs::Marker visited_marker = setup_marker(1);
  visited_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  visited_marker.color.g = 1.0;
  // visited_marker.header.frame_id = "world";
  // visited_marker.header.stamp = ros::Time();
  // visited_marker.id = 1;
  // visited_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  // visited_marker.color.a = 1.0;
  // visited_marker.color.g = 1.0;
  // visited_marker.scale.x = 0.01;
  // visited_marker.scale.y = 0.01;
  // visited_marker.scale.z = 0.01;
  // visited_marker.pose.orientation.w = 1.0;
  visualization_msgs::Marker marker = setup_marker(0, 0.005);
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  // marker.header.frame_id = "world";
  // marker.header.stamp = ros::Time();
  // marker.id = 0;
  // marker.color.a = 1.0;
  // marker.scale.x = 0.005;
  // marker.scale.y = 0.005;
  // marker.scale.z = 0.005;
  // marker.pose.orientation.w = 1.0;
  
  bool has_reached = false;	// Used to determine final stop condition

  moveit_planner::MoveJoint moveJoint;
  moveJoint.request.val = std::vector<double>{0.0, -0.78, 0.0, -0.78, 0.0, 0.78, 0.785398};
  moveJoint.request.execute = true;
  move_joint_client.call(moveJoint);

  // Get image
  ROS_INFO("Receiving first image");
  ros::Subscriber img_sub = nh.subscribe("/panda_camera/depth/points", 1, imgCallback);
  recv_img();

  // Process
  // Downsample
  // ros_pcl_manip::Downsample downsample;
  // downsample.request.cloud = _img;
  // downsample.request.size = 0.001;
  // downsample_client.call(downsample);
  // _img = downsample.response.cloud;
  // Filter
  ROS_INFO("Fitting initial curve");
  // scrap_burning::CurveFitting curve_fitting = filter_fit();
  scrap_burning::FilterPath filter_path;
  filter_path.request.cloud = _img;
  filter_path.request.field_name = "r";
  filter_path.request.field_lower = 10;
  filter_path.request.field_upper = 255;
  seg_client.call(filter_path);
  ROS_INFO("Filtered path");
  // Fit
  scrap_burning::CurveFitting curve_fitting;
  curve_fitting.request.filtered_cloud = filter_path.response.filtered_cloud;
  curve_fitting.request.control_points = 20; // Original: 20
  curve_fitting.request.order = 3;
  curve_fitting.request.smoothness = 100.0;
  curve_fitting.request.radius = 10.0;
  curve_fitting.request.min_dist = 0.01;
  curve_fitting.request.hide_viewer = true;
  fit_client.call(curve_fitting);
  ROS_INFO("Fit curve");

  // Main loop of follow-fit
  // Initially, we need to head to a point on the fit path, for now we go to the first (for no particular reason)
  ROS_INFO("Moving to initial pose");
  std::vector<geometry_msgs::Pose> visited_poses;
  geometry_msgs::Pose initial_pose = toTargetPose(curve_fitting.response.sampled_points[0],
						  curve_fitting.response.sampled_normals[0]);
  visited_poses.push_back(initial_pose);
  // Now, construct the MoveAway msg that will be used constantly
  moveit_planner::MoveAway move_away;
  move_away.request.distance = 0.15;
  move_away.request.execute = true;
  // Move to the first point initially
  move_away.request.pose = initial_pose;
  move_away_client.call(move_away);
  curve_fitting.request.control_points = 10;
  // This is the threshold beneath which we stop measuring points
  // If the maximum distance is less than this, stop follwing
  // double thresh = 0.001;
  // Now we begin the loop
  while(true) {
    // First thing to do is acquire an image
    _img = sensor_msgs::PointCloud2();
    while(_img.data.size() == 0)
      ros::spinOnce();
    // recv_img();

    // Now, filter + curve fit
    // curve_fitting = filter_fit();
    filter_path.request.cloud = _img;
    seg_client.call(filter_path);
    curve_fitting.request.filtered_cloud = filter_path.response.filtered_cloud;
    fit_client.call(curve_fitting);

    // Visualize the fit points
    marker.points = curve_fitting.response.sampled_points;
    visited_marker.points = std::vector<geometry_msgs::Point>();
    for(int i = 0; i < visited_poses.size(); ++i)
      visited_marker.points.push_back(visited_poses[i].position);
    // vis_pub.publish(marker);
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    vis_pub.publish(visited_marker);
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    // Now that the curve is fit, we need to decide on which point to go towards
    // For now, we go towards the farthest point from the one we were on previously
    // Not ideal, we really just want to follow a line here, but this is a good enough approximation for now
    // The farthest does not work exactly, we need the farthest ON AVERAGE point
    // Loop over all points in the fit curve_img = sensor_msgs::PointCloud2();

    int chosen_pt = 0;
    double min_dist = 0.0;
    for(int fit_pt = 0; fit_pt < curve_fitting.response.sampled_points.size(); ++fit_pt) {
      double total_dist = 0.0;
      // Loop over all previously visited points
      for(int prev_pt = 0; prev_pt < visited_poses.size(); ++prev_pt)
	total_dist += get_dist(visited_poses[prev_pt].position, curve_fitting.response.sampled_points[fit_pt]);
      Line a; Line b;
      if(visited_poses.size() > 1) {
	a.a = visited_poses[visited_poses.size() - 1 - 1].position;
	a.b = visited_poses[visited_poses.size() - 1].position;
	b.a = visited_poses[visited_poses.size() - 1].position;
	b.b = curve_fitting.response.sampled_points[fit_pt];
      }
      if(total_dist > min_dist && (visited_poses.size() == 1 || dotProd(a, b) > 0)) {
	chosen_pt = fit_pt;
	min_dist = total_dist;
      }
    }

    double last_pt_dist = get_dist(visited_poses[visited_poses.size() - 1].position,
				   curve_fitting.response.sampled_points[chosen_pt]);
    double thresh = 0.0001;
    ROS_INFO_STREAM("Distance is " << min_dist << " total, from the last point it is "
		    << last_pt_dist);
    if(last_pt_dist <= thresh) {
      if(has_reached)		// DONE
	return 0;
      has_reached = true;	// Reached the end once
      ROS_INFO("Reached below threshold distance, returning to other branch");
      // Now that we have reached the end, let's return to the initial point
      move_away.request.pose = visited_poses[0];
      move_away_client.call(move_away);
      // Now we fit a curve, and move in the opposite direction to the first two points
      ros::Duration pre_img_wait(1.0);
      pre_img_wait.sleep();
      _img = sensor_msgs::PointCloud2();
      while(_img.data.size() == 0)
	ros::spinOnce();
      filter_path.request.cloud = _img;
      seg_client.call(filter_path);
      curve_fitting.request.filtered_cloud = filter_path.response.filtered_cloud;
      fit_client.call(curve_fitting);
      marker.points = curve_fitting.response.sampled_points;
      vis_pub.publish(marker);
      ros::spinOnce();
      ros::spinOnce();
      ros::spinOnce();
      ros::spinOnce();
      // recv_img();
      // curve_fitting = filter_fit();
      // Get the opposite point
      geometry_msgs::Point pt_a = visited_poses[0].position;
      geometry_msgs::Point pt_b = visited_poses[1].position;
      // Get farthest point
      double max_return_dist = 0.0;
      int max_return_index = 0;
      for(int i = 0; i < curve_fitting.response.sampled_points.size(); ++i) {
	double cur_dist = get_dist(pt_b, curve_fitting.response.sampled_points[i]);
	Line l1, l2;
	l1.a = pt_b;
	l1.b = pt_a;
	l2.a = pt_a;
	l2.b = curve_fitting.response.sampled_points[i];
	if(cur_dist > max_return_dist) {
	  max_return_dist = cur_dist;
	  max_return_index = i;
	}
      }
      // Now we move, then we keep the loop going
      chosen_pt = max_return_index;
    }

    // Now that we decided on a point to go towards, we go and repeat the loop above
    move_away.request.pose = toTargetPose(curve_fitting.response.sampled_points[chosen_pt],
					  curve_fitting.response.sampled_normals[chosen_pt]);
    target_marker.pose.position = move_away.request.pose.position;
    vis_pub.publish(target_marker);
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    std::cout << "Move to target location?" << std::endl;
    char move = 'n';
    std::cin >> move;
    // Do nothing, just wait for input
    if(!move_away_client.call(move_away)) {
      ROS_INFO("Could not move to target pose, trying again");
    }
    else			// Point is now visited
      visited_poses.push_back(move_away.request.pose);

    scrap_burning::PCLConcatRequest concat_req;
    concat_req_client.call(concat_req);
  }

  return 0;
}
