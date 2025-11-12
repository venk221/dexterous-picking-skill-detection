#include <functional>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

// #include "octomap_msgs/conversions.h"
// #include "octomap_msgs/Octomap.h"
#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"

class TFGetter {
public:
  TFGetter()
    : _tf() {}

  octomath::Pose6D getTransform(const std::string& base, const std::string& frame) {
    tf::StampedTransform trans;
    for(unsigned i = 0; i < MAX_ATTEMPTS; ++i) {
      try {
	_tf.lookupTransform(base, frame, ros::Time(0), trans);
	return octomath::Pose6D(octomath::Vector3(trans.getOrigin().getX(),
						  trans.getOrigin().getY(),
						  trans.getOrigin().getZ()),
				octomath::Quaternion(trans.getRotation().getW(), trans.getRotation().getX(),
						     trans.getRotation().getY(), trans.getRotation().getZ()));
      }
      catch(tf::TransformException ex) {
	ROS_WARN_STREAM("Failed to get transform: " << ex.what());
      }
    }

    return octomath::Pose6D(octomath::Vector3(0, 0, 0), octomath::Quaternion(1, 0, 0, 0));
  }
private:
  constexpr static unsigned MAX_ATTEMPTS = 3;

  tf::TransformListener _tf;
};

class OctomapConcat {
public:
  OctomapConcat(double res, std::function<octomath::Pose6D()> pFunc, const std::string &filepath)
    : _tree(res), _pFunc(pFunc), _filepath(filepath) {}

  void addCloud(const sensor_msgs::PointCloud2 &pc2) {
    // auto cloud = _pointCloud2ToOctomap(pc2);
    // auto colorVector = _pointCloud2ToColorVector(pc2);

    // // Insert points
    // _tree.insertRay({0, 0, 0}, );
    // _tree.insertPointCloud(cloud, octomath::Vector3(), _pFunc());

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2, "z");
 
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
	_tree.insertRay({0, 0, 0}, {*iter_x, *iter_y, *iter_z}, 255, 255, 255);
    }
    // // Color the added points
    // for(std::size_t i = 0; i < colorVector.size(); ++i)
    //   _tree.setNodeColor(cloud[i].x(), cloud[i].y(), cloud[i].z(),
    // 			 255, 0, 0);
  }
  void addLineCloud(const sensor_msgs::PointCloud2 &pc2) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2, "z");
 
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
	_tree.insertRay({0, 0, 0}, {*iter_x, *iter_y, *iter_z}, 255, 0, 0);
    }
  }
  void saveMap() const {
    std::ofstream ofs(_filepath);
    _tree.write(ofs);
  }
private:
  octomap::ColorOcTree _tree;

  std::function<octomath::Pose6D()> _pFunc;

  std::string _filepath;

  octomap::Pointcloud _pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud) {
    octomap::Pointcloud octomapCloud;
    octomapCloud.reserve(cloud.data.size() / cloud.point_step);
 
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
 
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
	octomapCloud.push_back(*iter_x, *iter_y, *iter_z);
    }

    return octomapCloud;
  }
  typedef octomap::ColorOcTreeNode::Color Color;
  std::vector<Color> _pointCloud2ToColorVector(const sensor_msgs::PointCloud2 &cloud) {
    std::vector<Color> ret;

    for(sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(cloud, "r"), iter_g(cloud, "g"), iter_b(cloud, "b"); iter_r != iter_r.end(); ++iter_r, ++iter_g, ++iter_b) {
      ret.push_back(Color(*iter_r, *iter_g, *iter_b));
    }

    return ret;
  }
};
