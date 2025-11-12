#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <chrono>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include "scrap_burning/active_vision/camera.hpp"
#include "scrap_burning/active_vision/grid.hpp"

// Octomap concat msgs
#include "scrap_burning/RecordRequest.h"
#include "scrap_burning/AddToRecord.h"

#include "moveit_planner/MoveAway.h"
#include "visualization_msgs/Marker.h"

#include "include/octomap/octomap.h"
#include "include/octomap/ColorOcTree.h"

using scrap_burning::active_vision::PositionedCell;
using scrap_burning::active_vision::RayCells;
using scrap_burning::active_vision::Map;
using scrap_burning::active_vision::Node;
using scrap_burning::active_vision::Point;
using Dim = scrap_burning::active_vision::Camera::Dim;
using scrap_burning::active_vision::Ray;

using scrap_burning::active_vision::getShortestDist;
using scrap_burning::active_vision::draw;
using scrap_burning::active_vision::OctoRay;

constexpr double PI = 3.1415926535;
constexpr Dim WIDTH=320;
constexpr Dim HEIGHT=240;
constexpr double FOCAL_LENGTH=554.254691191187;
constexpr double RAY_LENGTH=0.7;
// Alpha used in the occProb calculation for unknown cells based on the distance from frontier cells
constexpr double ALPHA = 2.0;
// The standard deviation of the camera's depth noise
constexpr double STD_DEV = 0.01;
constexpr double UNKNOWN_CELL_STD_DEV=1000.0;
// How much to expand each frontier cell to generate the search space
// This is PER DIRECTION (+EXP in x, -EXP in x, +EXP in y, -EXP in y, etc...)
constexpr unsigned EXP=20;

// We store a set of voxels here

// Custom typedefs
typedef std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> KeyMap;
typedef std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> KeySet;

// Nearest Neighbour counts that define what a frontier cell is
constexpr scrap_burning::active_vision::NeighbourCount NN_THRESH{10, 6, 40};
// Color that defines what the line voxels are
const octomap::ColorOcTreeNode::Color LINE_COLOR{255, 0, 0};

std::vector<octomap::OcTreeKey> genGrid(std::vector<octomap::OcTreeKey> start, int cnt) {
  KeySet existingCells;
  existingCells.insert(start.begin(), start.end());
  std::vector<octomap::OcTreeKey> next;
  for(int i = 0; i < cnt; ++i) {
    // Expand each cell in start towards a new direction
    for(const octomap::OcTreeKey &key : start) {
      for(int dX = -1; dX <= 1; ++dX)
	for(int dY = -1; dY <= 1; ++dY)
	  for(int dZ = -1; dZ <= 1; ++dZ) {
	    octomap::OcTreeKey dKey(key[0] + dX, key[1] + dY, key[2] + dZ);
	      if(existingCells.find(dKey) == existingCells.end()) { // New key, not encountered before
		existingCells.insert(dKey);
		next.push_back(dKey);
	      }
	  }
    }

    // Swap and reset
    std::swap(next, start);
    // Clear it for insertion on the next loop if we will iterate again
    if(i < cnt - 1)
      next.clear();
  }

  return next;
}

// Compute the quality of a ray r using Map tree
// The previous variance is the current map variances
// frontier is the list of frontier cells
double computeRayQuality(const OctoRay &r, const Map *tree, const std::unordered_map<Node*, double> &prevVariance,
			 const RayCells &frontier, const KeyMap &prevQuality) {
  double ret = 0.0;
  double prevViewProb = 1.0;

  float dt = scrap_burning::active_vision::getDiscStepVal(r, tree);
  for(float t = 0.0; t <= RAY_LENGTH; t += dt) {
    octomap::point3d pos = r(t);
    octomap::ColorOcTreeNode *loc = tree->search(pos);
    // Calculate occupancy probability
    double occProb = 0.0;
    double prevCellVar = UNKNOWN_CELL_STD_DEV;
    if(loc == NULL) {
      // Unknown cell, get its distance from the nearest frontier cell
      auto iter = prevQuality.find(tree->coordToKey(pos));
      if(iter == prevQuality.end())
	occProb = 0.0;
      else
	occProb = iter->second;
    }
    else if(loc->getOccupancy() == 1.0) // Early termination, the ray hits a wall here
      return ret;
    else {
      occProb = loc->getOccupancy();
      auto varIt = prevVariance.find(loc);
      if(varIt != prevVariance.end())
	prevCellVar = varIt->second;
    }

    // Set prevViewProb for next iteration
    prevViewProb *= (1 - occProb);
    
    // Calculate entropy
    double infGain = 0.5 * log(1 + (prevCellVar * prevCellVar) / (STD_DEV * STD_DEV));
    ret += (infGain * occProb * prevViewProb);
  }

  return ret;
}

KeyMap computeQualitySphere(const scrap_burning::active_vision::Camera &cam,
			    const Map *tree, std::unordered_map<Node*, double> &prevVariance,
			    const RayCells &frontier,
			    const KeyMap &prevQuality) {
  octomap::OcTreeKey cameraPosition = cam.getDiscretePosition(tree);
  octomap::point3d ocPtCameraPosition = tree->keyToCoord(cameraPosition);
  // Point ptCameraPosition(ocPtCameraPosition.x(), ocPtCameraPosition.y(), ocPtCameraPosition.z());
  // Get a list of sphere voxels
  std::vector<octomap::OcTreeKey> sphereVoxels = scrap_burning::active_vision::sphereCast(cameraPosition, RAY_LENGTH,
											  tree);
  std::cout << "Sphere cast" << std::endl;
  // Shoot a ray towards each sphere voxel and start computing the quality
  std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> qualMap;
  for(const octomap::OcTreeKey &key : sphereVoxels) {
    // octomap::point3d keyPt(tree->keyToCoord(key));
    OctoRay r(ocPtCameraPosition, tree->keyToCoord(key));
    // Ray r(ptCameraPosition, Eigen::Vector3f{keyPt.x(), keyPt.y(), keyPt.z()});
    qualMap[key] = computeRayQuality(r, tree, prevVariance, frontier, prevQuality);
  }
  std::cout << "Computed qualities" << std::endl;

  return qualMap;
}

KeyMap::const_iterator getSphereKey(const Point &pt, const KeyMap &qualMap, const Map *tree) {
  octomap::OcTreeKey key = tree->coordToKey(pt[0], pt[1], pt[2]);
  KeyMap::const_iterator iter = qualMap.find(key);
  if(iter == qualMap.end()) {
    // Check neighbours
    for(std::size_t dX = key[0] - 1; dX <= key[0] + 1; ++dX) {
      for(std::size_t dY = key[1] - 1; dY <= key[1] + 1; ++dY) {
  	for(std::size_t dZ = key[2] - 1; dZ <= key[2] + 1; ++dZ) {
  	  iter = qualMap.find(octomap::OcTreeKey(dX, dY, dZ));
  	  if(iter != qualMap.end())
  	    return iter;
  	}
      }
    }
  }

  return iter;
}

double evalViewpoint(const scrap_burning::active_vision::Camera &cam, const Map *tree,
		     const KeyMap &qualMap) {
  double ret = 0.0;
  Dim w = cam.getDescription().width;
  Dim h = cam.getDescription().height;
  int cellFailCount = 0;
  for(Dim x = 0; x < w; ++x) {
    for(Dim y = 0; y < h; ++y) {
      auto qualIter = getSphereKey(cam.cast(x, y)(RAY_LENGTH), qualMap, tree);
      if(qualIter != qualMap.end())
	ret += qualIter->second;
    }
  }
  return ret;
}

std::pair<scrap_burning::active_vision::Transform, double> getBestViewpoint(scrap_burning::active_vision::Camera cam, const Map *tree, const KeyMap &qualMap) {
  scrap_burning::active_vision::Transform best = cam.getTransform();
  double bestQual = 0.0;
  // Loop over all voxels on the sphere, point to each, and calculate
  std::size_t cnt = qualMap.size();
  std::size_t cur = 0;
  KeyMap evaledDirs;
  for(const std::pair<octomap::OcTreeKey, double> &voxel : qualMap) {
    std::cout << cur << " / " << cnt << '\n';
    cur++;
    // Point the camera
    octomap::point3d pt(tree->keyToCoord(voxel.first));
    Ray dir(Point(0, 0, 0), Point(pt.x(), pt.y(), pt.z()));
    auto iter = getSphereKey(dir(RAY_LENGTH), qualMap, tree);
    auto evaledIter = evaledDirs.find(iter->first);
    // Already evaluated
    if(evaledIter != evaledDirs.end())
      continue;

    // Not evaluated, compute and store
    cam.pointAt(dir.getVector());
    double score = evalViewpoint(cam, tree, qualMap);
    evaledDirs[iter->first] = score;
    if(score > bestQual) {
      best = cam.getTransform();
      bestQual = score;
    }
  }

  return {best, bestQual};
}

double evalViewpoint(const scrap_burning::active_vision::Camera &cam, const Map *tree,
		     std::unordered_map<Node*, double> &prevVariance) {
  double ret = 0.0;

  RayCells frontier = scrap_burning::active_vision::filterNNC(tree, NN_THRESH, LINE_COLOR);
  Dim w = cam.getDescription().width;
  Dim h = cam.getDescription().height;
  for(Dim x = 0; x < w; ++x) {
    for(Dim y = 0; y < h; ++y) {
      RayCells cells(scrap_burning::active_vision::discretize(cam.cast(x, y), tree, RAY_LENGTH));
      // Store the previous cell's viewProb
      double prevViewProb = 1.0;
      for(const auto &cell : cells) {
	// Calculate occupancy probability
	double occProb = 0.0;
	if(cell.second == NULL) {
	  // Unknown cell, get its distance from the nearest frontier cell
	  double dist = getShortestDist(cell, frontier);
	  occProb = exp(-1 * pow(2 * dist, 2));
	}
	else
	  occProb = cell.second->getOccupancy();

	// Calculate probability of viewing this cell
	double viewProb = prevViewProb * (1 - occProb);
	// Set prevViewProb for next iteration
	prevViewProb = viewProb;

	// Calculate entropy
	double prevCellVar = 0.0;
	auto varIt = prevVariance.find(cell.second);
	if(cell.second == NULL || varIt == prevVariance.end()) // If it hasn't been measured before, set the variance really high
	  prevCellVar = UNKNOWN_CELL_STD_DEV;
	else
	  prevCellVar = varIt->second;
	double curVariance = 1 / (1 / prevCellVar + 1 / STD_DEV);
	double infGain = 0.5 * log(1 + (prevCellVar * prevCellVar) / (STD_DEV * STD_DEV));
	double weightedInfGain = infGain * occProb * viewProb;
	ret += weightedInfGain;
      }
    }
  }

  return ret;
}

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "active_vision_node");
  ros::NodeHandle nh;

  // Create buffers to start storing transforms
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Load the first octree
  ros::ServiceClient begOctomap = nh.serviceClient<scrap_burning::RecordRequest>("begin_octomap");
  ros::ServiceClient addOctomap = nh.serviceClient<scrap_burning::AddToRecord>("add_octomap");

  // Load octree
  const std::string PATH("/home/fadi/scrap_ws/octreeFiltered.ot");
  Map *tree = dynamic_cast<Map*>(Map::read(PATH));

  // Construct Camera
  const scrap_burning::active_vision::Camera::CameraDescription desc{WIDTH, HEIGHT, FOCAL_LENGTH};
  Eigen::Matrix4f initPose;
  initPose <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;
  scrap_burning::active_vision::Transform initLoc(initPose);
  scrap_burning::active_vision::Camera initCam(desc, initLoc);
  // Construct intial variance map
  auto start = std::chrono::high_resolution_clock::now();
  std::unordered_map<Node*, double> varMap;
  for(Dim x = 0; x < initCam.getDescription().width; ++x) {
    for(Dim y = 0; y < initCam.getDescription().height; ++y) {
      RayCells cells(scrap_burning::active_vision::discretize(initCam.cast(x, y), tree, RAY_LENGTH));
      for(const auto &cell : cells) varMap[cell.second] = STD_DEV;
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto dur = std::chrono::duration<double, std::milli>(end - start);
  std::cout << "Execution time: " << dur.count() << std::endl;

  Eigen::Matrix4f locTrans = Eigen::Matrix4f::Identity();
  scrap_burning::active_vision::Transform loc(locTrans);
  scrap_burning::active_vision::Camera c(desc, loc);

  RayCells frontier = scrap_burning::active_vision::filterNNC(tree, NN_THRESH, LINE_COLOR);
  // Append to variance map additional probabilistic information on the frontier cells
  KeyMap probMap;
  for(const auto &cell : frontier) {
    octomap::OcTreeKey key = tree->coordToKey(cell.first[0], cell.first[1], cell.first[2]);
    for(int dX = -1; dX <= 1; dX += 1)
      for(int dY = -1; dY <= 1; dY += 1)
  	for(int dZ = -1; dZ <= 1; dZ += 1) {
  	  octomap::OcTreeKey dKey(key[0] + dX, key[1] + dY, key[2] + dZ);
  	  if(tree->search(dKey) == NULL) { // Unknown cell
  	    octomap::point3d pt = tree->keyToCoord(dKey);
  	    double dist = getShortestDist({Point(pt.x(), pt.y(), pt.z()), NULL}, frontier);
  	    probMap[dKey] = exp(-1 * pow(dist, 2));
  	  }
  	}
  }

  // Generate the cells we are searching over
  const double cellDelta = tree->getResolution();
  // KeySet searchSpace;
  std::vector<octomap::OcTreeKey> initialSeed;
  for(const auto &cell : frontier)
    initialSeed.push_back(tree->coordToKey(cell.first[0], cell.first[1], cell.first[2]));
  std::vector<octomap::OcTreeKey> searchSpace = genGrid(initialSeed, EXP);

  double topScore(0.0);
  scrap_burning::active_vision::Transform topTrans(Eigen::Matrix4f::Identity());
  std::size_t cnt(0);
  scrap_burning::active_vision::Transform tf(c.getTransform());
  Point topPos;
  Point topFacing;
  for(const auto &key : searchSpace) {
    // Print current progress
    std::cout << cnt << "/" << searchSpace.size() << '\n';
    ++cnt;

    // If the key is in unknown or occupied space, skip this
    auto node = tree->search(key);
    if(node == NULL || node->getOccupancy() >= 0.5)
      continue;
    auto cnt = scrap_burning::active_vision::getNNC(tree, tree->keyToCoord(key));
    if(std::get<scrap_burning::active_vision::OCCUPIED>(cnt) + std::get<scrap_burning::active_vision::UNKNOWN>(cnt) != 0) // All cells nearby are known free
      continue;

    // Place camera in this location
    octomap::point3d ocPt(tree->keyToCoord(key));
    Point p(ocPt.x(), ocPt.y(), ocPt.z());
    tf.translation() = p;
    c.setTransform(tf);
    // Compute quality sphere
    KeyMap sphere = computeQualitySphere(c, tree, varMap, frontier, probMap);
    // Point camera at each frontier cell
    for(const auto &cell : frontier) {
      c.pointAt(cell.first);
      // Evaluate the viewpoint at this location
      double score(evalViewpoint(c, tree, sphere));
      if(score > topScore) {
  	topScore = score;
  	topTrans = c.getTransform();
  	topFacing = cell.first;
  	topPos = p;
      }
    }
  }
  std::cout << "Place camera at " << topPos << " and face " << topFacing << std::endl;
  std::cout << "Best quality: " << topScore;
  std::cout << "Setting topTrans to " << topTrans.matrix() << std::endl;

  Point loc2 = c.getTransform().translation();
  RayCells centerCells;
  draw(discretize(c.cast(0, 0), tree, RAY_LENGTH), tree);
  draw(discretize(c.cast(WIDTH, 0), tree, RAY_LENGTH), tree);
  draw(discretize(c.cast(0, HEIGHT), tree, RAY_LENGTH), tree);
  draw(discretize(c.cast(WIDTH, HEIGHT), tree, RAY_LENGTH), tree);

  tree->write("/home/fadi/scrap_ws/camera_sim_opt.ot");

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_camera_optical_link";
  pose.pose.position.x = 0.235;
  pose.pose.position.y = 0.355;
  pose.pose.position.z = 0.845;
  // pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.4129013;
  pose.pose.orientation.y = -0.6401032;
  pose.pose.orientation.z = 0.3466545;
  pose.pose.orientation.w = 0.5473674;

  geometry_msgs::PointStamped center;
  center.header.frame_id = "panda_camera_optical_link";
  center.point.x = 0.235;
  center.point.y = 0.355;
  center.point.z = 0.845;
  geometry_msgs::PointStamped target;
  target.header.frame_id = center.header.frame_id;
  target.point.x = -0.075;
  target.point.y = -0.315;
  target.point.z = 0.725;
  while(true) {
    try {
      tfBuffer.transform(center, center, "world");
      tfBuffer.transform(target, target, "world");
      // Transform was a success
      break;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Failed to transform, trying again");
      ros::Duration(0.1).sleep();
    }
  }
  center.header.frame_id = "world";
  target.header.frame_id = "world";
  // pose.header.frame_id = "world";

  moveit_planner::MoveAway move_away;
  move_away.request.pose = pose.pose;
  move_away.request.distance = 0.1;
  move_away.request.execute = true;
  ros::ServiceClient move_away_client = nh.serviceClient<moveit_planner::MoveAway>("move_away_point");
  move_away_client.call(move_away);

  visualization_msgs::Marker targetMarker;
  targetMarker.type = visualization_msgs::Marker::SPHERE;
  targetMarker.action = visualization_msgs::Marker::ADD;
  targetMarker.id = 0;
  targetMarker.pose = pose.pose;
  targetMarker.scale.x = 0.1;
  targetMarker.scale.y = 0.1;
  targetMarker.scale.z = 0.1;
  targetMarker.color.a = 1.0;
  targetMarker.header.frame_id = "panda_link0";
  targetMarker.header.stamp = ros::Time();
  ros::Publisher visPub = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 0);
  visPub.publish(pose);
  ros::Publisher ptPub = nh.advertise<geometry_msgs::PointStamped>("center_pose", 0);
  ros::Publisher ptPub2 = nh.advertise<geometry_msgs::PointStamped>("point_pose", 0);

  while(ros::ok()) {
    ros::spinOnce();
    visPub.publish(pose);
    ptPub.publish(center);
    ptPub2.publish(target);
    ros::Duration(0.1).sleep();
  }

  return 0;
}
