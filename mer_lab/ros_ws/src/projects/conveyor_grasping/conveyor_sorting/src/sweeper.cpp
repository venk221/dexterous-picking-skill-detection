#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "moveit_planner/MovePose.h"
#include "moveit_planner/MoveCart.h"

#include "conveyor_sorting_msgs/Sweep.h"
#include "conveyor_sorting/config.hpp"
#include "conveyor_sorting/pt_utils.hpp"

#include <qnd/vis.hpp>
#include <qnd/conv.hpp>
#include <qnd/geom.hpp>

ros::ServiceClient cartClient{};
ros::ServiceClient poseClient{};

qnd::Visualizer* visPtr{nullptr};

bool executeSweeping(conveyor_sorting_msgs::Sweep::Request& req,
		     conveyor_sorting_msgs::Sweep::Response& res) {
  moveit_planner::MovePose startPose;
  moveit_planner::MoveCart cart;

  // Offset the points
  static constexpr double zOffset{0.135};
  static constexpr double perpOffset{0.305 / 2};
  static constexpr double zRotation{-0.25 * M_PI};
  // We can do the regular z offset here
  req.sweeping_action.start.z += zOffset;
  req.sweeping_action.end.z += zOffset;

  geometry_msgs::Pose start;
  geometry_msgs::Pose end;

  // First we need to compute the angle of the end effector
  // This should remain constant as we sweep across
  // We can assume first of all that the end effector has the y axis horizontal
  Eigen::Matrix3d rotMat(3, 3);
  // The z axis is always downwards
  rotMat(0, 2) = 0.0;
  rotMat(1, 2) = 0.0;
  rotMat(2, 2) = -1.0;
  // For the z axis, we need to first calculate the sweeping direction
  double deltaX = req.sweeping_action.end.x - req.sweeping_action.start.x;
  double deltaY = req.sweeping_action.end.y - req.sweeping_action.start.y;
  // The y axis is aligned with the above direction
  geometry_msgs::Point yDir{qnd::normalize(qnd::createPt(deltaX, deltaY, 0.0))};
  // If deltaY > 0 (right to left) flip yDir (Keeps sweeping side consistently close to the robot)
  if(deltaY > 0)
    yDir = qnd::multiply(yDir, -1.0);
  rotMat(0, 1) = yDir.x;
  rotMat(1, 1) = yDir.y;
  rotMat(2, 1) = 0.0;
  // The x is a cross product with y and z
  geometry_msgs::Point xDir{qnd::crossProd(yDir, qnd::createPt(0.0, 0.0, -1.0))};
  rotMat(0, 0) = xDir.x;
  rotMat(1, 0) = xDir.y;
  rotMat(2, 0) = xDir.z;

  // The x axis is along the direction we will offset by
  req.sweeping_action.start = qnd::add(req.sweeping_action.start, qnd::multiply(xDir, perpOffset));
  req.sweeping_action.end = qnd::add(req.sweeping_action.end, qnd::multiply(xDir, perpOffset));

  // Take the above output and rotate it along its z axis by 45 degrees
  Eigen::AngleAxis<double> postTrans(zRotation, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d postTransMat;
  postTransMat = postTrans;

  rotMat = postTransMat * rotMat;

  // Compute a quaternion from the rotation matrix above
  Eigen::Quaterniond quat(rotMat);

  start = qnd::createPose(req.sweeping_action.start,
			  qnd::createQuat(quat.x(), quat.y(), quat.z(), quat.w()));
  end = qnd::createPose(req.sweeping_action.end,
			qnd::createQuat(quat.x(), quat.y(), quat.z(), quat.w()));

  visPtr->visPose(10, start, "world", qnd::createPt(0.05, 1.0, 0.0));

  startPose.request.val = start;
  startPose.request.execute = req.execute;
  poseClient.call(startPose);

  cart.request.scale = req.scale;
  cart.request.execute = req.execute;
  cart.request.val.push_back(start);
  cart.request.val.push_back(end);

  return cartClient.call(cart);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sweeper_node");
  ros::NodeHandle nh{};

  ConfigLoader cfgLoader{nh};
  cfgLoader.loadParams();

  ros::ServiceServer sweepingServer{nh.advertiseService(cfgLoader.getConfig().sweeper.sweep_topic, executeSweeping)};
  cartClient = nh.serviceClient<moveit_planner::MoveCart>("cartesian_move");
  poseClient = nh.serviceClient<moveit_planner::MovePose>("move_to_pose");

  visPtr = new qnd::Visualizer(nh);

  ros::spin();

  return 0;
}
