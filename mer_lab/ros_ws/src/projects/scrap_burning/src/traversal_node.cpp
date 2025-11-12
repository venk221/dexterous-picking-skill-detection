#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

#include "scrap_burning/Traverse.h"
#include "moveit_planner/MoveAway.h"
#include "moveit_planner/MoveCart.h"

geometry_msgs::Pose _to_pose(const geometry_msgs::Point& point,
			     const geometry_msgs::Point& normal,
			     bool flip) {
  geometry_msgs::Pose ret;
  // Position is unchanged
  ret.position = point;

  Eigen::Matrix3d trans(3, 3);
  Eigen::Matrix3d postTrans(3, 3);
  postTrans << 0.707, -0.707, 0,
    0.707,  0.707, 0,
    0,          0, 1;
  // Rotation - z
  // Decide how to flip
  int iflip = flip ? -1 : 1;
  trans(0, 2) = normal.x * iflip;
  trans(1, 2) = normal.y * iflip;
  trans(2, 2) = normal.z * iflip;
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
  // q = q.normalized();
  ret.orientation.x = q.x();
  ret.orientation.y = q.y();
  ret.orientation.z = q.z();
  ret.orientation.w = q.w();

  return ret;
}

ros::ServiceClient moveAway;
ros::ServiceClient moveCart;

bool trav(scrap_burning::Traverse::Request& req,
          scrap_burning::Traverse::Response& res) {
  ROS_INFO("Received request");
  std::vector<geometry_msgs::Pose> poses;
  moveit_planner::MoveCart moveCartReq;
  for(int i = 0; i < req.pos.size(); ++i) {
    // Create pose and send request
    auto pose = _to_pose(req.pos[i], req.norm[i], true);
    moveit_planner::MoveAway moveAwayReq;
    moveAwayReq.request.pose = pose;
    // moveAwayReq.request.distance = 0.175;
    moveAwayReq.request.distance = req.dist;
    moveAwayReq.request.execute = false;
    ROS_INFO("Calling movement node");
    moveAway.call(moveAwayReq);
    poses.push_back(moveAwayReq.response.awayPose);
  }
  moveCartReq.request.val = poses;
  moveCartReq.request.execute = true;
  moveCart.call(moveCartReq);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traversal_node");
  ros::NodeHandle nh;

  ros::ServiceServer travServ = nh.advertiseService("traverse_path", trav);
  moveAway = nh.serviceClient<moveit_planner::MoveAway>("move_away_point");
  moveCart = nh.serviceClient<moveit_planner::MoveCart>("cartesian_move");

  ros::spin();

  return 0;
}
