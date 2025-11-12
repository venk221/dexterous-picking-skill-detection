#ifndef SCRAP_BURNING_PATH_HPP
#define SCRAP_BURNING_PATH_HPP

#include <iostream>

#include <geometry_msgs/Pose.h>

namespace scrap_burning {
  class Path {
  public:
    Path(const std::string &base, const std::string &target)
      : _buffer(), _listener(_buffer), _base(base), _target(target), _poses() {}

    bool addPoint() {
      try {
	geometry_msgs::TransformStamped tf = _buffer.lookupTransform(_base, _target, ros::Time(0));
	_poses.push_back(_toPose(tf.transform));
      } catch(tf2::TransformException &ex) {
	return false;
      }

      return true;
    }

    friend std::ostream &operator<<(std::ostream &os, const Path &path) {

      for(const auto &pose : path._poses) {
	os << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ", ";
	os << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << '\n';
      }

      return os;
    }
  private:
    // TF2 stuff
    tf2_ros::Buffer _buffer;
    tf2_ros::TransformListener _listener;

    // Coordinate frames
    std::string _base;
    std::string _target;

    // A list of all poses so far
    std::vector<geometry_msgs::Pose> _poses;

    geometry_msgs::Pose _toPose(const geometry_msgs::Transform trans) {
      geometry_msgs::Pose ret;

      ret.position.x = trans.translation.x;
      ret.position.y = trans.translation.y;
      ret.position.z = trans.translation.z;

      ret.orientation = trans.rotation;

      return ret;
    }
  };
}

#endif
