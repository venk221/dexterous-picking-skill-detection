// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// scrap_burning includes
#include "scrap_burning/timer.hpp"
#include "scrap_burning/FilterPath.h"
#include "scrap_burning/BeginActive.h"
#include "scrap_burning/CurveFitting.h"
#include "scrap_burning/RecordRequest.h"
#include "scrap_burning/AddTrajWaypoint.h"
#include "scrap_burning/ComputeNextBest.h"
#include "scrap_burning/config/config.hpp"
#include "scrap_burning/PCLConcatRequest.h"
#include "scrap_burning/StopRecordingTraj.h"
#include "scrap_burning/StartRecordingTraj.h"

// moveit_planner includes
#include "moveit_planner/GetPose.h"
#include "moveit_planner/MoveAway.h"
#include "moveit_planner/MoveCart.h"
#include "moveit_planner/MoveJoint.h"
#include "moveit_planner/SetParams.h"
#include "moveit_planner/AddAttachedCollision.h"

// ros_pcl_manip includes
#include "ros_pcl_manip/ToFile.h"
#include "ros_pcl_manip/Downsample.h"
#include "ros_pcl_manip/OutlierRemoval.h"

// C++ includes
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>

#define PI 3.14159265

using PointNormal = std::pair<geometry_msgs::Point, geometry_msgs::Point>;
using PointNormalScore = std::tuple<geometry_msgs::Point, geometry_msgs::Point, double>;

geometry_msgs::Point normalize(const geometry_msgs::Point& vec) {
  geometry_msgs::Point ret;

  double mag = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
  ret.x = vec.x / mag;
  ret.y = vec.y / mag;
  ret.z = vec.z / mag;

  return ret;
}

class PathFollowClass {
public:
  PathFollowClass(ros::NodeHandle& nh) : _nh(nh), _config(_nh) {
    reload_params();
    _setup_comm();
  };
  void reload_params() {
    if(!_config.loadParams()) {
      ROS_ERROR("Failed to load one or more parameters");
      throw "Failed to load parameters";
    }
  }
  bool attempt_follow() {
    // Clear variables
    _path_end_indices.clear();
    _path_points.clear();
    _path_normals.clear();

    // Uncomment to add in torch as collision object
    // _add_collision_obj();

    // Open file handle if saving required
    if(_config.save_points)
      _ofs.open(_config.save_points_path);

    // Initialize active vision service if needed
    ROS_INFO("Checking active vision");
    if(_config.followMethod == scrap_burning::CurveFollowMethod::ACTIVE_VISION) {
      ROS_INFO("Computing active vision");
      scrap_burning::BeginActive ba;
      ba.request.octomap_res = _config.octomap_res;
      ba.request.cam_res_x = _config.camera_x;
      ba.request.cam_res_y = _config.camera_y;
      ba.request.cam_focal_length = _config.camera_focal_length;
      ba.request.ray_length = _config.ray_length;
      ba.request.frontier_bbx_size = _config.frontier_bounding_box;
      ba.request.threads = _config.threads;
      _active_start_client.call(ba);
    }

    // Setup moveit parameters
    if(!_setupMoveitParams()) {
      ROS_WARN("Failed to set moveit params");
      return false;
    }

    // Start timing
    scrap_burning::Timer timer;

    // First, move to the initial joint goal
    moveit_planner::MoveJoint move_joint;
    move_joint.request.val = _config.initial_joint_vals;
    move_joint.request.execute = true;
    if(!_move_joint_client.call(move_joint)) {
      ROS_ERROR("Could not move to initial pose");
      _ofs.close();
      return false;
    }
    ROS_INFO("Moved to initial pose");

    // Start recording
    _startRecordingTraj();

    // Get and store the current robot pose
    moveit_planner::GetPose get_pose;
    if(!_get_pose_client.call(get_pose)) {
      ROS_ERROR("Could not get pose from moveit_planner");
      _ofs.close();
      return false;
    }
    // Record starting time
    ros::Time start = ros::Time::now();
    // Empty pcl concat request
    scrap_burning::PCLConcatRequest concat_req;
    // Store pose for future use
    geometry_msgs::Pose cur_pose = get_pose.response.pose;
    // Copy it for backtracking
    geometry_msgs::Pose ini_pose = get_pose.response.pose;

    _concat_req_client.call(concat_req);

    // Get an initial point
    // Fit a curve
    std::vector<PointNormalScore> candidates = _get_next_points();

    // Once an initial curve was fit, we go to the first point there
    // All calls to _attempt_move will override our current pose
    if(!_attempt_move(candidates, cur_pose.position)) {
      ROS_INFO("Did not move to target, aborting");
      _ofs.close();
      return false;		// Could not move to target, stop here
    }

    // Add to list of visited points
    std::vector<geometry_msgs::Pose> visited_poses; // visited poses
    _get_pose_client.call(get_pose);
    visited_poses.push_back(get_pose.response.pose);
    // Concatenate
    _concat_req_client.call(concat_req);

    // Initialize whether we backtracked or not
    bool back_flag = false;

    // Add a lap (since we moved once)
    timer.addLap();

    // Now we can begin the loop
    while(true) {
      // Visualize if enabled
      // Would be faster if we store this message outside and add points to it if _view_visited
      // But we don't visit enough points for it to make a big difference, so keep it
      if(_config.view_visited) {
        visualization_msgs::Marker visited_marker = _setup_marker(1, 0.01);
        visited_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        visited_marker.color.g = 1.0;
	for(const auto &pose : visited_poses)
	  visited_marker.points.push_back(pose.position);
        _vis_pub.publish(visited_marker);
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
      }

      // Img->Filter->Fit
      candidates = _get_next_points();

      // No candidates left, loop done
      if(candidates.size() == 0) {
	ROS_INFO("No candidates left, scanning done");
	std::ofstream ofs("/tmp/timing.txt");
	timer.output(ofs);

	// Output trajectory
	_outputTrajectory("/tmp/traj.txt");

	return true;
      }
      ROS_INFO_STREAM("Top score: " << std::get<2>(candidates[0]));

      // Otherwise, move as normal
      if(!_attempt_move(candidates, cur_pose.position)) {
	ROS_INFO("Could not move to target");
	_ofs.close();
	return false;
      }

      // Record current pose
      _get_pose_client.call(get_pose);
      visited_poses.push_back(get_pose.response.pose);

      // Concatenate points
      _concat_req_client.call(concat_req);

      // Add new lap
      timer.addLap();
    }
  }

  // Callbacks, do not call these directly, only ROS should
  void img_sub_callback(const sensor_msgs::PointCloud2::ConstPtr& img) {
    _cur_img = (*img);
  }
private:
  ros::NodeHandle& _nh;
  scrap_burning::ScrapBurningConfig _config;

  // Service Clients
  ros::ServiceClient _seg_client;
  ros::ServiceClient _fit_client;
  ros::ServiceClient _concat_req_client;
  ros::ServiceClient _move_param_client;
  ros::ServiceClient _move_joint_client;
  ros::ServiceClient _move_away_client;
  ros::ServiceClient _move_cart_client;
  ros::ServiceClient _get_pose_client;
  ros::ServiceClient _save_img_client;
  ros::ServiceClient _downsample_client;
  ros::ServiceClient _outlier_removal_client;
  ros::ServiceClient _active_start_client;
  ros::ServiceClient _active_add_client;
  ros::ServiceClient _start_recording;
  ros::ServiceClient _add_traj_waypoint;
  ros::ServiceClient _stop_recording;
  ros::ServiceClient _add_attached_coll_client;

  // Publishers
  ros::Publisher _vis_pub;
  ros::Publisher _seg_pub;

  // Subscribers
  ros::Subscriber _img_sub;

  // Path variables
  std::vector<int> _path_end_indices;
  std::vector<geometry_msgs::Point> _path_points;
  std::vector<geometry_msgs::Point> _path_normals;

  // Other variables
  sensor_msgs::PointCloud2 _cur_img;
  std::string _cur_img_frame;
  std::ofstream _ofs;

  // Helper method to load a specific service
  template <typename T>
  void _load_service(const std::string &serviceName, ros::ServiceClient &client) {
    ROS_INFO_STREAM("Waiting for service " << serviceName << "...");
    ros::service::waitForService(serviceName, ros::Duration(_config.service_wait_timeout));

    client = _nh.serviceClient<T>(serviceName);
  }
  
  void _setup_comm() {
    _load_service<scrap_burning::FilterPath>("path_filter", _seg_client);
    _load_service<scrap_burning::CurveFitting>("fit_curve", _fit_client);
    _load_service<scrap_burning::PCLConcatRequest>("concat_server", _concat_req_client);
    _load_service<moveit_planner::SetParams>("set_params", _move_param_client);
    _load_service<moveit_planner::MoveJoint>("move_to_joint_space", _move_joint_client);
    _load_service<moveit_planner::MoveAway>("move_away_point", _move_away_client);
    _load_service<moveit_planner::MoveCart>("cartesian_move", _move_cart_client);
    _load_service<moveit_planner::GetPose>("get_pose", _get_pose_client);
    _load_service<ros_pcl_manip::ToFile>("save_to_pcd", _save_img_client);
    _load_service<ros_pcl_manip::Downsample>("downsample", _downsample_client);
    _load_service<ros_pcl_manip::OutlierRemoval>("outlier_removal", _outlier_removal_client);
    _load_service<scrap_burning::BeginActive>("start_active_vision", _active_start_client);
    _load_service<scrap_burning::ComputeNextBest>("add_active_vision", _active_add_client);
    _load_service<scrap_burning::StartRecordingTraj>("start_recording_traj", _start_recording);
    _load_service<scrap_burning::AddTrajWaypoint>("add_traj_waypoint", _add_traj_waypoint);
    _load_service<scrap_burning::StopRecordingTraj>("stop_recording_traj", _stop_recording);
    _load_service<moveit_planner::AddAttachedCollision>("add_attached_collision", _add_attached_coll_client);

    // Setup all publishers
    _vis_pub = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    _seg_pub = _nh.advertise<sensor_msgs::PointCloud2>("segmented_path", 0);

    // Setup all subscribers
    _img_sub = _nh.subscribe(_config.depth_topic, 1, &PathFollowClass::img_sub_callback, this);
  }
  bool _setupMoveitParams() {
    ROS_INFO("Setting moveit params");
    moveit_planner::SetParams params;

    params.request.velScaling = 0.0; // Velocity is unchanged
    params.request.planningTime = _config.planning_time;

    return _move_param_client.call(params);
  }
  void _get_fresh_img() {
    _cur_img = sensor_msgs::PointCloud2(); // Remove current image
    while(_cur_img.data.size() == 0)	   // Keep spinning until we get a new image
      ros::spinOnce();
    // Store image source frame
    _cur_img_frame = _cur_img.header.frame_id;
  }
  sensor_msgs::PointCloud2 _filter_cur_img(const sensor_msgs::PointCloud2& cloud) {
    scrap_burning::FilterPath filter_path;
    if(cloud.data.size() == 0) // No data
      return sensor_msgs::PointCloud2(); // Return empty
    
    filter_path.request.cloud = cloud;
    filter_path.request.field_name = _config.filter_field;
    filter_path.request.field_lower = _config.filter_lower;
    filter_path.request.field_upper = _config.filter_upper;
    filter_path.request.secondary_threshold = _config.filter_secondary_thresh;

    if(!_seg_client.call(filter_path)) {
      ROS_ERROR("Could not call segmentation client for path filtering");
      return sensor_msgs::PointCloud2(); // Return empty
    }

    return filter_path.response.filtered_cloud;
  }
  sensor_msgs::PointCloud2 _downsample_cur_img(const sensor_msgs::PointCloud2& cloud) {
    ros_pcl_manip::Downsample downsample;
    if(cloud.data.size() == 0)	// No data
      return sensor_msgs::PointCloud2(); // Return empty
    downsample.request.cloud = cloud;
    downsample.request.size = _config.downsample_leaf_size;
    if(!_downsample_client.call(downsample)) {
      ROS_ERROR("Could not call downsample service");
      return sensor_msgs::PointCloud2();
    }

    return downsample.response.cloud;
  }
  sensor_msgs::PointCloud2 _remove_outliers(const sensor_msgs::PointCloud2& cloud) {
    ros_pcl_manip::OutlierRemoval outlier_removal;
    if(cloud.data.size() == 0)
      return sensor_msgs::PointCloud2();
    outlier_removal.request.cloud = cloud;
    outlier_removal.request.mean_k = _config.mean_k;
    outlier_removal.request.stddev_thresh = _config.stddev_thresh;
    if(!_outlier_removal_client.call(outlier_removal)) {
      ROS_ERROR("Could not call outlier removal service");
      return sensor_msgs::PointCloud2();
    }

    return outlier_removal.response.filtered_cloud;
  }
  scrap_burning::CurveFitting _fit_curve(const sensor_msgs::PointCloud2& filtered_cloud,
					 const sensor_msgs::PointCloud2& full_cloud,
					 const std::string& frame_id) {
    scrap_burning::CurveFitting curve_fitting;
    if(full_cloud.data.size() == 0)	// No data
      return curve_fitting;
    curve_fitting.request.filtered_cloud = filtered_cloud;
    curve_fitting.request.full_cloud = full_cloud;
    // Set headers
    curve_fitting.request.filtered_cloud.header.frame_id = frame_id;
    curve_fitting.request.full_cloud.header.frame_id = frame_id;
    curve_fitting.request.control_points = _config.control_pts;
    curve_fitting.request.order = 3;
    curve_fitting.request.smoothness = _config.smoothness;
    curve_fitting.request.radius = _config.radius;
    curve_fitting.request.min_dist = _config.min_cutoff_dist;
    curve_fitting.request.k = _config.normal_nn_count;
    curve_fitting.request.hide_viewer = true;
    curve_fitting.request.use_skeleton = (_config.followMethod == scrap_burning::CurveFollowMethod::SKELETON);
    curve_fitting.request.skeleton_leaf_size = _config.skel_leaf_size;

    if(!_fit_client.call(curve_fitting))
      ROS_ERROR("Could not call curve fitting service");

    return curve_fitting;
  }
  // Setup a marker with some better defaults
  visualization_msgs::Marker _setup_marker(int id, double scale=0.01) {
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
  // Replace void with scrap_burning::CurveFitting to use the points in the final stage
  scrap_burning::CurveFitting _global_fit() {
    // Obtain concatenated point cloud
    scrap_burning::PCLConcatRequest global_req;
    _concat_req_client.call(global_req);
    // We now have the global image, we can filter/fit
    sensor_msgs::PointCloud2 filt_glob = _filter_cur_img(global_req.response.concat_cloud);
    ros_pcl_manip::ToFile glob_file, filt_file;
    glob_file.request.cloud = global_req.response.concat_cloud;
    glob_file.request.filepath = "/tmp/globalCloud.pcd";
    filt_file.request.cloud = filt_glob;
    filt_file.request.filepath = "/tmp/filteredGlobal.pcd";
    _save_img_client.call(glob_file);
    _save_img_client.call(filt_file);
    // Send request to fit a curve (using skeletons, it does not work with curve fitting)
    // bool prev_val = _use_pcl_nurbs;
    // _use_pcl_nurbs = false;
    // scrap_burning::CurveFitting ret = _fit_curve(filt_glob, global_req.response.concat_cloud, _cur_img_frame);
    // _use_pcl_nurbs = prev_val;
    // return ret;
    return scrap_burning::CurveFitting();
  }
  std::vector<PointNormalScore> _get_next_points() {
    std::vector<PointNormalScore> ret;

    _get_fresh_img();		// Will reload the current image to get a new one
    ROS_INFO_STREAM("Obtained image with " << _cur_img.data.size() << " points");

    // Check if outlier removal is enabled
    if(_config.mean_k > 0) {
      ROS_INFO("Removing outliers");
      _cur_img = _remove_outliers(_cur_img); // Override the current image
    }

    // Check if downsampling is enabled
    if(_config.downsample_leaf_size > 0.0) {
      ROS_INFO("Downsampling cloud");
      _cur_img = _downsample_cur_img(_cur_img);	// Override the current image
    }

    // Filter the image
    ROS_INFO_STREAM("Filtering point cloud with " << _cur_img.data.size() << " points");
    sensor_msgs::PointCloud2 filtered_cloud = _filter_cur_img(_cur_img);
    ROS_INFO_STREAM("Filtered image has " << filtered_cloud.data.size() << " points");
    // Check if need to publish segmented cloud
    if(_config.view_segmented_curve) {
      ROS_INFO("Publishing segmented curve");
      _seg_pub.publish(filtered_cloud);
      ros::spinOnce();
      ros::spinOnce();
      ros::spinOnce();
    }
    // Check if need to save cloud
    if(_config.save_filtered) {
      ros_pcl_manip::ToFile to_file;
      to_file.request.cloud = filtered_cloud;
      to_file.request.filepath = _config.save_filtered_path;
      if(_save_img_client.call(to_file))
	ROS_INFO_STREAM("Saved filtered path to file " << _config.save_filtered_path);
      else
	ROS_WARN_STREAM("Could not save filtered cloud to file " << _config.save_filtered_path);
      to_file.request.cloud = _cur_img;
      to_file.request.filepath = "/tmp/base.pcd";
      _save_img_client.call(to_file);
    }

    // Check what to do based on the curve follow method
    switch(_config.followMethod.type) {
    case scrap_burning::CurveFollowMethod::PCL_NURBS:
    case scrap_burning::CurveFollowMethod::SKELETON: {
      ROS_INFO("Fitting curve");
      scrap_burning::CurveFitting curve_pts = _fit_curve(filtered_cloud, _cur_img, _cur_img_frame);
      for(int i = 0; i < curve_pts.response.sampled_points.size(); ++i)
	ret.push_back({curve_pts.response.sampled_points[i], curve_pts.response.sampled_normals[i], 1.0});

      // If visualization is enabled, publish the curve as well
      if(_config.view_curve) {
	ROS_INFO("Visualizing data");
	visualization_msgs::Marker curve_markers = _setup_marker(0, 0.005);
	curve_markers.type = visualization_msgs::Marker::SPHERE_LIST;
	curve_markers.points = curve_pts.response.sampled_points;
	_vis_pub.publish(curve_markers);
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
      }
    }
      break;
    default:			// Active vision
      ROS_INFO("Calling Active Vision Method");
      scrap_burning::ComputeNextBest cnb;
      cnb.request.cloud = _cur_img;
      cnb.request.filtered_cloud = filtered_cloud;
      cnb.request.optimized = _config.optimized;
      if(!_active_add_client.call(cnb)) {
	ROS_ERROR("Failed to get next view from active vision, has it been initialized?");
	return ret;
      }
      for(int i = 0; i < cnb.response.position.size(); ++i)
	ret.push_back({cnb.response.position[i], cnb.response.normal[i], cnb.response.score[i]});
      break;
    }

    return ret;
  }

  void _add_collision_obj() {
    moveit_msgs::AttachedCollisionObject attachedObj;
    attachedObj.link_name = "panda_link8";
    attachedObj.object.header.frame_id = "panda_link8";
    attachedObj.object.id = "3dPrinted";

    geometry_msgs::Pose pose;
    pose.orientation.w = sin(PI / 8);
    pose.orientation.z = cos(PI / 8);
    pose.position.z = 0.15 / 2;
    pose.position.x = 0.53 / 2 / 1.41 - 0.17 / 1.41;
    pose.position.y = -0.53 / 2 / 1.41 + 0.17 / 1.41;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.53;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.13;

    attachedObj.object.primitives.push_back(primitive);
    attachedObj.object.primitive_poses.push_back(pose);
    attachedObj.object.operation = attachedObj.object.ADD;

    moveit_planner::AddAttachedCollision msg;
    msg.request.collObject = attachedObj;

    _add_attached_coll_client.call(msg);
  }

  // Returns the farthest point in cur_pts from prev_pts
  // ensure_dp_cont ensures the dot product is positive with the last 2 pts
  int _get_farthest_point(const std::vector<PointNormal>& prev_pts,
			  const std::vector<PointNormal>& cur_pts,
			  bool ensure_dp_cont) {
    const std::vector<PointNormal> &iterPts = cur_pts;

    int ret = 0;
    double min_dist = 0.0;
    bool found_valid = false;
    for(int fit_pt = 0; fit_pt < iterPts.size(); ++fit_pt) {
      double total_dist = 0.0;
      for(int prev_pt = 0; prev_pt < prev_pts.size(); ++prev_pt)
	total_dist += _get_dist(iterPts[fit_pt].first, prev_pts[prev_pt].first);
      // This point is the furthest so far
      if(total_dist > min_dist) {
	// No need to get dot product
	if(!ensure_dp_cont || prev_pts.size() <= 1) {
	  ret = fit_pt;
	  min_dist = total_dist;
	  found_valid = true;
	}
	// Need to get dot product
	else if(_dot_prod(prev_pts[prev_pts.size() - 1 - 1].first, prev_pts[prev_pts.size() - 1].first,
			  prev_pts[prev_pts.size() - 1].first, iterPts[fit_pt].first) > 0) {
	  ret = fit_pt;
	  min_dist = total_dist;
	  found_valid = true;
	}
      }
    }

    // If no points were valid, return -1
    return found_valid ? ret : -1;
  }

  // Returns L2 distance between 2 points
  inline double _get_dist(const geometry_msgs::Point& a,
			  const geometry_msgs::Point& b) {
    return (a.x - b.x) * (a.x - b.x) +
      (a.y - b.y) * (a.y - b.y) +
      (a.z - b.z) * (a.z - b.z);
  }

  // Returns dot product between two lines l1 and l2
  double _dot_prod(const geometry_msgs::Point& l1_a, const geometry_msgs::Point& l1_b,
		   const geometry_msgs::Point& l2_a, const geometry_msgs::Point& l2_b) {
    geometry_msgs::Point v1;
    v1.x = l1_b.x - l1_a.x;
    v1.y = l1_b.y - l1_a.y;
    v1.z = l1_b.z - l1_a.z;
    geometry_msgs::Point v2;
    v2.x = l2_b.x - l2_a.x;
    v2.y = l2_b.y - l2_a.y;
    v2.z = l2_b.z - l2_a.z;

    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
  }

  geometry_msgs::Pose _to_pose(const geometry_msgs::Point& point,
			       const geometry_msgs::Point& normal) {
    geometry_msgs::Pose ret;
    // Position is unchanged
    ret.position = point;

    Eigen::Matrix3d trans(3, 3);
    Eigen::Matrix3d postTrans(3, 3);
    postTrans << 0.707, -0.707, 0,
                 0.707,  0.707, 0,
                 0,          0, 1;
    // postTrans <<
    //   -1,  0, 0,
    //   0,  -1, 0,
    //   0,   0, 1;
    // Rotation - z
    // Decide how to flip
    trans(0, 2) = normal.x;
    trans(1, 2) = normal.y;
    trans(2, 2) = normal.z;
    // Rotation - y
    geometry_msgs::Point y_axis;
    y_axis.z = 0;
    // Calculate ay, ax, b, cy, cx
    // Initialize variables
    double nx{trans(0, 2)}, ny{trans(1, 2)}, nz{trans(2, 2)};
    double den = sqrt(nx*nx + ny*ny);
    double y{-nx/den};
    double x{ny/den};
    int flip_sign = y > 0 ? -1 : 1;
    // Flip signs if y is negative
    x = x * flip_sign;
    y = y * flip_sign;
    // Done, insert values
    trans(0, 1) = x;
    trans(1, 1) = y;
    trans(2, 1) = 0.0;		// Horizontal
    trans.col(0) = -trans.col(2).cross(trans.col(1));
    // Convert to quaternion
    Eigen::Quaterniond q(trans * postTrans);
    // Eigen::Quaterniond q(trans);
    // q = q.normalized();
    ret.orientation.x = q.x();
    ret.orientation.y = q.y();
    ret.orientation.z = q.z();
    ret.orientation.w = q.w();

    return ret;
  }

  // Attempts to move to target location
  // Pose moved towards is obtained by passing the value to target_pt
  // cur_pt is where the robot is now, used to orient the normals
  // HATE how this was written, but we gotta stick with it for now
  bool _attempt_move(const std::vector<PointNormalScore> &point_normals,
		     geometry_msgs::Point& target_pt) {
    for(const auto &point_normal : point_normals) {
      const geometry_msgs::Point &pos = std::get<0>(point_normal);
      const geometry_msgs::Point &norm = std::get<1>(point_normal);
      const double score = std::get<2>(point_normal);
      
      // Create pose
      geometry_msgs::Point normalized_norm = normalize(norm);
      geometry_msgs::Pose pose = _to_pose(pos, normalized_norm);

      // Check if visualization is required
      if(_config.view_target) {
	visualization_msgs::Marker target_marker = _setup_marker(2, 0.01);
	target_marker.type = visualization_msgs::Marker::SPHERE;
	target_marker.color.b = 1.0;
	target_marker.pose.position = pose.position;
	_vis_pub.publish(target_marker);

	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
      }
      if(_config.view_target_norm) {
	visualization_msgs::Marker norm_marker = _setup_marker(3, 0.01);
	norm_marker.type = visualization_msgs::Marker::SPHERE;
	norm_marker.color.b = 1.0;

	// Calculate normal position
	geometry_msgs::Point norm_pos;
	norm_pos.x = pos.x + norm.x * _config.distance_from_scrap;
	norm_pos.y = pos.y + norm.y * _config.distance_from_scrap;
	norm_pos.z = pos.z + norm.z * _config.distance_from_scrap;
	norm_marker.pose.position = norm_pos;
	_vis_pub.publish(norm_marker);

	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
      }

      // Construct message
      moveit_planner::MoveAway move_away;
      move_away.request.distance = _config.distance_from_scrap;
      move_away.request.execute = false;
      move_away.request.pose = pose;

      // Whether to execute or not
      bool execute = true;
      if(_config.interactive_execution) {
	// Simulate movement
	if(!_move_away_client.call(move_away)) {
	  ROS_WARN("Failed to plan to pose, planning to next one");
	  continue;
	}
	char usr_input = 'n';
	std::cout << "Move to target pose? [y -> move]\n";
	std::cin >> usr_input;

	execute = usr_input == 'y'; // Should we execute?
      }

      if(!execute)		// Do not execute
	return false;

      // Execute motion
      move_away.request.execute = true;
      bool res = _move_away_client.call(move_away);
      if(res) {		// Can move
	target_pt = move_away.response.awayPose.position;
	ROS_INFO("Successfully moved to new pose");

	// Record the new waypoint
	_recordWaypoint();

	// Record visited point if required
	if(_config.save_points) {
	  _ofs << pos.x << ',' << pos.y << ',' << pos.z;
	  _ofs << '\n';
	  _ofs << norm.x << ',' << norm.y << ',' << norm.z;
	  _ofs << '\n';
	  _ofs << score;
	  _ofs << '\n';
	}
	return true;
      }
      else {
	ROS_WARN("Failed to plan to target position, going to next point");
	continue;
      }
    }

    return false;		// Failed to execute motion
  }

  void _startRecordingTraj() {
    scrap_burning::StartRecordingTraj msg;

    msg.request.base = "panda_link0";
    msg.request.target = "panda_link8";
    msg.request.rate = 10;

    _start_recording.call(msg);
  }

  void _recordWaypoint() {
    scrap_burning::AddTrajWaypoint msg;

    _add_traj_waypoint.call(msg);
  }

  void _outputTrajectory(const std::string &filepath) {
    scrap_burning::StopRecordingTraj msg;

    msg.request.filepath = filepath;

    _stop_recording.call(msg);
  }
};
