#include <cmath>
#include <vector>
#include <thread>
#include <future>
#include <fstream>
#include <algorithm>

#include <ros/ros.h>
#include <tf2/convert.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include "scrap_burning/active_vision/grid.hpp"
#include "scrap_burning/active_vision/camera.hpp"
#include "scrap_burning/active_vision/active.hpp"

#include "scrap_burning/BeginActive.h"
#include "scrap_burning/ComputeNextBest.h"

#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"

#include "scrap_burning/pcl.hpp"

#include "visualization_msgs/Marker.h"

#include "qnd/vis.hpp"
#include "qnd/geom.hpp"

// Typedefs
using scrap_burning::pcl::PointType;
using scrap_burning::pcl::CloudType;
using scrap_burning::pcl::CloudPtr;
using scrap_burning::active_vision::Camera;
using CameraDescription = scrap_burning::active_vision::Camera::CameraDescription;
using scrap_burning::active_vision::PositionedCell;
using scrap_burning::active_vision::RayCells;
using scrap_burning::active_vision::Map;
using scrap_burning::active_vision::Node;
using scrap_burning::active_vision::Point;
using Dim = scrap_burning::active_vision::Camera::Dim;
using scrap_burning::active_vision::Ray;
using scrap_burning::active_vision::discretize;

using scrap_burning::active_vision::getShortestDist;
using scrap_burning::active_vision::draw;
using scrap_burning::active_vision::OctoRay;
using scrap_burning::active_vision::Cluster;

using KeyMap = scrap_burning::active_vision::KeyMap;
using KeySet = scrap_burning::active_vision::KeySet;

using PointNormal = std::pair<geometry_msgs::Point, geometry_msgs::Point>;
using PointNormalScore = std::tuple<geometry_msgs::Point, geometry_msgs::Point, double>;

class ActiveVision {
public:
  ActiveVision(ros::NodeHandle &nh, double octomapRes, CameraDescription desc, double rayLength, unsigned bbxSize, unsigned threads)
    : _nh(nh), _tree(octomapRes), _cam(desc), _rayLength(rayLength), _bbxSize(bbxSize), _threads(threads) {
    // _tree.setProbHit(0.9);
    // _tree.setProbMiss(0.1);
  }

  void save(const std::string &filepath, std::pair<geometry_msgs::Point, geometry_msgs::Point> *camPosition=nullptr) {
    std::ofstream ofs(filepath);

    if(camPosition != nullptr) {
      // Create a copy of the current occupancy grid
      Map *treeCopy = _createCopy();

      // Draw the rays at the optimal camera location
      auto trans = _cam.getTransform();
      trans.translation() = Point(camPosition->first.x, camPosition->first.y, camPosition->first.z);
      _cam.setTransform(trans);
      Point target(camPosition->second.x, camPosition->second.y, camPosition->second.z);
      _cam.pointAt(target + trans.translation());

      draw(discretize(_cam.cast(0, 0), treeCopy, _rayLength), treeCopy);
      draw(discretize(_cam.cast(_cam.getDescription().width, 0), treeCopy, _rayLength), treeCopy);
      draw(discretize(_cam.cast(0, _cam.getDescription().height), treeCopy, _rayLength), treeCopy);
      draw(discretize(_cam.cast(_cam.getDescription().width, _cam.getDescription().height), treeCopy, _rayLength), treeCopy);

      treeCopy->write(ofs);

      delete treeCopy;
    }

    else
      _tree.write(ofs);
  }

  std::vector<PointNormalScore> getNextView(sensor_msgs::PointCloud2 cloud, sensor_msgs::PointCloud2 lineCloud, const std::string &frame_id, const std::string &base_id, bool optimized=true) {
    geometry_msgs::TransformStamped trans = _getTransform(cloud.header.frame_id);

    _transform(cloud, trans);
    _transform(lineCloud, trans);

    // Add the clouds
    octomap::point3d camOrigin{trans.transform.translation.x,
			       trans.transform.translation.y,
			       trans.transform.translation.z};
    ROS_INFO("Adding Clouds");
    _addCloud(cloud, camOrigin, 255, 255, 255);
    _addCloud(lineCloud, camOrigin, 255, 0, 0);

    // Save new tree
    std::ofstream ofs("/home/fadi/scrap_ws/test.ot");
    _tree.write(ofs);

    // Required values
    RayCells frontier;
    std::vector<Point> orientations;
    std::vector<octomap::OcTreeKey> searchSpace;

    // Optimized active vision
    if(optimized) {
      // Cluster frontier cells
      RayCells fullFrontier = _getFrontier();

      // Early termination
      if(fullFrontier.size() == 0)
	return {};

      std::vector<Cluster> clusters = _cluster(fullFrontier);
      std::vector<Point> clusterCentroids;
      for(const auto &cluster : clusters) {clusterCentroids.push_back(cluster.getCentroid());}
      // Pick closest centroid
      int targetCluster = scrap_burning::active_vision::getNearest(Point(camOrigin.x(), camOrigin.y(), camOrigin.z()),
								   clusterCentroids);
      // Insert target cluster's frontier cells
      for(const auto &clusterPt : clusters[targetCluster].getPts())
	frontier.push_back({clusterPt, _tree.search(clusterPt[0], clusterPt[1], clusterPt[2])});

      // Orientations are just the target centroid
      orientations.push_back(clusterCentroids[targetCluster]);
      // Search space is a sphere around the centroid
      searchSpace = scrap_burning::active_vision::sphereCast(_tree.coordToKey(clusterCentroids[targetCluster][0],
									      clusterCentroids[targetCluster][1],
									      clusterCentroids[targetCluster][2]),
							     _bbxSize *  _tree.getResolution(), &_tree);
    }
    // Naive active vision
    else {
      // The frontier cells are not filtered out
      frontier = _getFrontier();
      qnd::Visualizer vis(_nh);
      std::vector<geometry_msgs::Point> points{};
      std::cout << "Got " << frontier.size() << " frontier points\n";
      for(const auto& frontier_pt : frontier) {
	points.push_back(qnd::eigenToGeom(frontier_pt.first));
	std::cout << points.back().x << ", " << points.back().y << ", " << points.back().z << '\n';
      }
      std::cout << "Visualizing data\n";
      vis.visPoint(100, points, "world", qnd::createPt(0.01), 1.0f);
      ros::spinOnce();
      ros::spinOnce();
      ros::spinOnce();

      // Early termination
      if(frontier.size() == 0)
	return {};

      // The orientations are all the frontier cells
      for(const auto &frontierCell : frontier)
	orientations.push_back(frontierCell.first);

      // The search space is generated through an expanding bounding box around the frontier cells
      std::vector<octomap::OcTreeKey> initialSeed;
      for(const auto &frontierCell : frontier)
	initialSeed.push_back(_tree.coordToKey(frontierCell.first[0], frontierCell.first[1], frontierCell.first[2]));
      searchSpace = scrap_burning::active_vision::genGrid(initialSeed, _bbxSize);
      std::vector<geometry_msgs::Point> searchPoints{};
      for(const auto& key : searchSpace) {
	auto keyPos = _tree.keyToCoord(key);
	searchPoints.push_back(qnd::createPt(keyPos.x(), keyPos.y(), keyPos.z()));
      }
      vis.visPoint(1000, searchPoints, "world", qnd::createPt(0.005), 0.0f, 1.0f);
      ros::spinOnce();
      ros::spinOnce();
      ros::spinOnce();
    }

    // Filter out search space
    searchSpace.erase(std::remove_if(searchSpace.begin(), searchSpace.end(),
				     [this](const octomap::OcTreeKey &key) {
				       auto nCnt = scrap_burning::active_vision::getNNC(&_tree, key, scrap_burning::active_vision::Connectivity::Corner);
				       return (_tree.search(key) == NULL || std::get<1>(nCnt) > 0);
				     }),
		      searchSpace.end());

    Map *frontierMap = _createCopy();
    draw(frontier, frontierMap);
    std::ofstream frontierOFS("/home/fadi/scrap_ws/frontier.ot");
    frontierMap->write(frontierOFS);
    delete frontierMap;

    // Calculate occupancy probability
    KeyMap probMap = _computeOccProb(frontier);

    // Filter out all non-trivial values of probMap
    RayCells proxVol;
    for(const auto &cell : probMap) {
      if(cell.second > 0.5) {
	std::cout << cell.second << std::endl;
	octomap::point3d pt(_tree.keyToCoord(cell.first));
	proxVol.push_back({Point(pt.x(), pt.y(), pt.z()), _tree.search(cell.first)});
      }
    }
    Map *volMap = _createCopy();
    draw(proxVol, volMap);
    std::ofstream volOfs("/home/fadi/scrap_ws/vol.ot");
    volMap->write(volOfs);

    RayCells ssc;
    for(const auto &key : searchSpace) {
      auto keyPos = _tree.keyToCoord(key);
      Point p(keyPos.x(), keyPos.y(), keyPos.z());
      ssc.push_back({p, _tree.search(keyPos)});
    }
    Map *bbMap = _createCopy();
    draw(ssc, bbMap);
    std::ofstream sss("/home/fadi/scrap_ws/ss.ot");
    bbMap->write(sss);
    delete bbMap;

    // Store point, normal, score, and distance from camera in this vector
    std::vector<std::tuple<geometry_msgs::Point, geometry_msgs::Point, double, double>> vals;
    // Output is stored here
    std::vector<PointNormalScore> ret;

    // Store result of threads here
    std::vector<std::future<std::vector<PointNormalScore>>> futureVals;
    double delta = searchSpace.size() / _threads;
    ROS_INFO_STREAM("Splitting " << searchSpace.size() << " positions into " << delta << " chunks across " << _threads << " threads");
    for(int i = 0; i < _threads; ++i)
      futureVals.push_back(std::async(std::launch::async, &ActiveVision::_getBestViews, this,
				      searchSpace.cbegin() + delta * i, searchSpace.cbegin() + delta * (i + 1),
				      probMap, orientations));

    // Store result of all threads here
    Point src(camOrigin.x(), camOrigin.y(), camOrigin.z());
    for(auto &futureVal : futureVals) {
      auto resultVector = futureVal.get();
      for(const auto &result : resultVector) {
	auto targetGeomPos = std::get<0>(result);
	Point target(targetGeomPos.x, targetGeomPos.y, targetGeomPos.z);
	// Push results as usual but compute distance as well
	vals.push_back({std::get<0>(result),
			std::get<1>(result),
			std::get<2>(result),
			(src - target).norm()});
      }
    }
    double totalScore = std::accumulate(vals.begin(), vals.end(), 0.0,
					[](double res, auto &val) {return res + std::get<2>(val);});
    double totalCost  = std::accumulate(vals.begin(), vals.end(), 0.0,
					[](double res, auto &val) {return res + std::get<3>(val);});
    ROS_INFO_STREAM("Total cost: " << totalCost);
    // Compute utility and store in ret
    for(const auto &val : vals) {
      ret.push_back({std::get<0>(val),
		     std::get<1>(val),
		     std::get<2>(val) / totalScore - 5.0 * std::get<3>(val) / totalCost});
    }

    std::sort(ret.begin(), ret.end(), [](auto &valA, auto &valB) {return std::get<2>(valA) > std::get<2>(valB);});

    return ret;
  }
private:
  ros::NodeHandle &_nh;
  octomap::ColorOcTree _tree;
  Camera _cam;
  double _rayLength;
  const unsigned _bbxSize;
  const unsigned _threads;

  static constexpr unsigned _MAX_ATTEMPTS = 3;

  Map *_createCopy() {
    std::ofstream tmp("/tmp/treeCopy.ot");
    _tree.write(tmp);
    tmp.close();
    std::ifstream tmpRead("/tmp/treeCopy.ot");
    Map *ret = dynamic_cast<Map*>(octomap::AbstractOcTree::read(tmpRead));
    return ret;
  }
  geometry_msgs::TransformStamped _getTransform(const std::string &frame_id) {
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener(buffer);
    geometry_msgs::TransformStamped trans;

    for(unsigned i = 0; i < _MAX_ATTEMPTS; ++i) {
      try {
	trans = buffer.lookupTransform("world", frame_id, ros::Time(0));
	return trans;
      } catch(tf2::TransformException &ex) {
	ROS_WARN_STREAM("Failed to get transform: " << ex.what());
	ros::Duration(2.5).sleep();
      }
    }

    ROS_WARN_STREAM("Failed to retrieve transform from " << "world" << " to " << frame_id);
    throw "Failed to get transform";
    return trans;
  }
  void _transform(sensor_msgs::PointCloud2 &cloud, const geometry_msgs::TransformStamped &trans) {
    tf2::doTransform(cloud, cloud, trans);
  }
  void _addCloud(const sensor_msgs::PointCloud2 &cloud, const octomap::point3d &camLoc,
		 unsigned char r, unsigned char g, unsigned char b) {
    CloudPtr pclCloud = scrap_burning::pcl::from_pc2(cloud);

    octomap::Pointcloud pc;
    for(const auto &point : *pclCloud) {
      // Skip if any NaNs were found
      if(isnan(point.x) || isnan(point.y) || isnan(point.z))
	continue;
      pc.push_back(point.x, point.y, point.z);
    }
    _tree.insertPointCloud(pc, r, g, b, camLoc, 1.5);
  }
  RayCells _getFrontier() {
    RayCells ret;
    std::vector<octomap::OcTreeKey> candidateCells;
    // Loop over all cells and get all free neighbours of the red ones
    for(auto it = _tree.begin_leafs(0), end=_tree.end_leafs(); it != end; ++it) {
      // Check color
      octomap::ColorOcTreeNode::Color c(it->getColor());
      if(c.r != 255 || c.g != 0 || c.b != 0)
	continue;

      // If it is a valid cell, get the neighbouring free cells
      std::vector<octomap::OcTreeKey> neighbours(scrap_burning::active_vision::getNeighbours(it.getKey(), scrap_burning::active_vision::Connectivity::Face));
      for(const auto &neighbourKey : neighbours) {
    	auto node = _tree.search(neighbourKey);
	if(node != NULL && !_tree.isNodeOccupied(node)) {
	  candidateCells.push_back(neighbourKey);
	}
      }
    }
    // With a list of free cells, filter those with no nearby unknown cells
    for(const auto &candidateFreeCellKey : candidateCells) {
      std::vector<octomap::OcTreeKey> neighbours(scrap_burning::active_vision::getNeighbours(candidateFreeCellKey, scrap_burning::active_vision::Connectivity::Face));
      auto nCnt = scrap_burning::active_vision::getNNC(&_tree, candidateFreeCellKey, scrap_burning::active_vision::Connectivity::Face);
      if(std::get<2>(nCnt) == 0)
	continue;
      for(const auto &neighbourKey : neighbours) {
	if(_tree.search(neighbourKey) == NULL) {
	  octomap::point3d pos = _tree.keyToCoord(candidateFreeCellKey);
	  ret.push_back({scrap_burning::active_vision::Point(pos.x(), pos.y(), pos.z()),
			 _tree.search(candidateFreeCellKey)});
	  break;
	}
      }
    }
    return ret;
  }
  std::vector<Cluster> _cluster(const RayCells &cells) {
    std::vector<Cluster> ret;

    // Convert above cells to keys
    std::vector<octomap::OcTreeKey> cellKeys;
    cellKeys.reserve(cells.size());
    for(const auto &cell : cells)
      cellKeys.push_back(_tree.coordToKey(cell.first[0],
					  cell.first[1],
					  cell.first[2]));

    // Store all processed cells here
    KeySet processed;
    for(const auto &cellKey : cellKeys) {
      // If this cell has been processed already skip it
      if(processed.find(cellKey) != processed.end())
	continue;
      processed.insert(cellKey);

      // Construct a cluster of neighbouring cells
      std::vector<octomap::OcTreeKey> cluster{cellKey};

      // Add all cells to the cluster
      // Store the current "frontier" of the neighbourhood in this vector
      std::vector<octomap::OcTreeKey> queue{cellKey};
      while(!queue.empty()) {
	auto curCell = queue.back();
	queue.pop_back();
	for(const auto &neighbour : scrap_burning::active_vision::getNeighbours(curCell, scrap_burning::active_vision::Connectivity::Corner)) {
	  if(std::find(cellKeys.begin(), cellKeys.end(), neighbour) != cellKeys.end() && processed.find(neighbour) == processed.end()) {
	    queue.push_back(neighbour);
	    cluster.push_back(neighbour);
	    processed.insert(neighbour);
	  }
	}
      }

      // Construct Cluster objects
      // Convert keys to points
      std::vector<Point> clusterPoints;
      clusterPoints.reserve(cluster.size());
      for(const auto &key : cluster) {
	auto octoPt = _tree.keyToCoord(key);
	clusterPoints.push_back(Point(octoPt.x(), octoPt.y(), octoPt.z()));
      }
      ret.push_back(Cluster(clusterPoints));
    }

    return ret;
  }
  KeyMap _computeOccProb(const RayCells &frontier) {
    KeyMap ret;

    // First, insert all cells belonging to the line into this tree
    for(auto it = _tree.begin_leafs(0), end=_tree.end_leafs(); it != end; ++it) {
      octomap::ColorOcTreeNode::Color c(it->getColor());
      if(c.r == 255 && c.g == 0 && c.b == 0)
	ret[it.getKey()] = it->getOccupancy();
    }

    // Start with the unknown cells neighbouring the frontier cells
    std::vector<octomap::OcTreeKey> queue;
    for(const auto &cell : frontier) {
      auto cellKey = _tree.coordToKey(cell.first[0], cell.first[1], cell.first[2]);
      for(const auto &neighbour : scrap_burning::active_vision::getNeighbours(cellKey, scrap_burning::active_vision::Connectivity::Face)) {
	if(_tree.search(neighbour) == NULL)
	  queue.push_back(neighbour);
      }
    }
    
    while(!queue.empty()) {
      octomap::OcTreeKey item = queue[0];
      queue.erase(queue.cbegin());
      for(const octomap::OcTreeKey &neighbour : scrap_burning::active_vision::getNeighbours(item, scrap_burning::active_vision::Connectivity::Face)) {
	// Check if it is unknown
	if(_tree.search(neighbour) != NULL)
	  continue;
	// Check if we didn't already add this cell
	if(ret.find(neighbour) != ret.end())
	  continue;
	
	octomap::point3d neighbourPt = _tree.keyToCoord(neighbour);
	double dist = getShortestDist({Point(neighbourPt.x(),
					     neighbourPt.y(),
					     neighbourPt.z()), NULL}, frontier);
	double score = exp(-1 * pow(dist * 5, 2));
	if(score < 0.1) continue;
	ret[neighbour] = score;
	queue.push_back(neighbour);
      }
    }
    return ret;
  }

  std::vector<PointNormalScore> _getBestViews(std::vector<octomap::OcTreeKey>::const_iterator cur, std::vector<octomap::OcTreeKey>::const_iterator end, const KeyMap &probMap, const std::vector<Point> &frontier) {
    int cnt = 0;
    std::size_t length = std::distance(cur, end);
    Camera cam(_cam);
    scrap_burning::active_vision::Transform tf(cam.getTransform());
    std::vector<PointNormalScore> ret;
    ROS_INFO_STREAM("Computing active vision scores across " << length << " candidate points");
    for(; cur != end; ++cur) {
      ROS_INFO_STREAM(cnt << "/" << length);
      ++cnt;

      // Place camera at given location
      octomap::point3d ocPt(_tree.keyToCoord(*cur));
      Point p(ocPt.x(), ocPt.y(), ocPt.z());
      tf.translation() = p;
      cam.setTransform(tf);

      KeyMap cache;
      for(const auto &cell : frontier) {
      	cam.pointAt(cell);
      	double score(scrap_burning::active_vision::evalViewpoint(cam, &_tree, cache, probMap, _rayLength));
	
      	geometry_msgs::Point pos, posFace;
      	pos.x = p[0]; pos.y = p[1]; pos.z = p[2];
      	posFace.x = cell[0] - p[0]; posFace.y = cell[1] - p[1]; posFace.z = cell[2] - p[2];
      	ret.push_back({pos, posFace, score});
      }
    }

    return ret;
  }
};

static bool initialized = false;
static ActiveVision *av = nullptr;
static ros::NodeHandle *nh = nullptr;

bool start(scrap_burning::BeginActive::Request &req,
	   scrap_burning::BeginActive::Response &res) {
  ROS_INFO("Starting active vision session");

  if(initialized) {
    ROS_INFO("Deleting old ActiveVision object");
    delete av;
  }

  ROS_INFO_STREAM("Creating pointer with resolution " << req.octomap_res << req.cam_res_x << req.cam_res_y << req.cam_focal_length << req.ray_length << req.frontier_bbx_size << req.threads);
  av = new ActiveVision(*nh, req.octomap_res, CameraDescription{req.cam_res_x, req.cam_res_y, req.cam_focal_length},
  			req.ray_length, req.frontier_bbx_size, req.threads);

  ROS_INFO("Active vision session initialized with given parameters");
  initialized = true;
  return true;
}

bool add(scrap_burning::ComputeNextBest::Request &req,
	 scrap_burning::ComputeNextBest::Response &res) {
  ROS_INFO("Adding new clouds to AV");
  if(!initialized) return false;

  ROS_INFO("Obtaining next best view");
  ROS_INFO_STREAM("Transforming from " << req.cloud.header.frame_id << " to " <<  "world");
  auto result = av->getNextView(req.cloud, req.filtered_cloud, req.cloud.header.frame_id, "world", req.optimized);

  for(const auto &pointnormalscore : result) {
    res.position.push_back(std::get<0>(pointnormalscore));
    res.normal.push_back(std::get<1>(pointnormalscore));
    res.score.push_back(std::get<2>(pointnormalscore));
  }

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "active_vision_node");
  nh = new ros::NodeHandle();

  ros::ServiceServer startAVServer = nh->advertiseService("start_active_vision", start);
  ros::ServiceServer addAVServer = nh->advertiseService("add_active_vision", add);

  ros::spin();

  return 0;
}
