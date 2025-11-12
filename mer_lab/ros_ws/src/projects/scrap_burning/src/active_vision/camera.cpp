#include "scrap_burning/active_vision/camera.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <unordered_set>

namespace scrap_burning {
  namespace active_vision {
    Camera::Camera(const CameraDescription &desc, const Transform &loc)
      : _desc(desc), _loc(loc) {}

    octomap::OcTreeKey Camera::getDiscretePosition(const Map *tree) const {
      return tree->coordToKey(_loc.translation()[0], _loc.translation()[1], _loc.translation()[2]);
    }

    Ray Camera::cast(Dim x, Dim y) const {
      // The image pane in the camera is _desc.focalLength pixels away
      // Since the ray is normalized anyway, we can just do everything in pixel coordinates
      Point pixelPt(-x + _desc.width / 2, -y + _desc.height / 2, _desc.focalLength);

      Ray camRay(Eigen::Vector3f{0, 0, 0}, pixelPt);
      camRay *= _loc;

      // Return the ray from the camera origin to the pixel in camera space
      return camRay;
    }

    RayCells Camera::getViewedCells(const Map *tree, double l) {
      RayCells ret;

      std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> keys;
      for(Dim x = 0; x < _desc.width; ++x)
	for(Dim y = 0; y < _desc.height; ++y)
	  for(const auto &pt : posDiscretize(cast(x, y), tree, l)) {
	    auto key = tree->coordToKey(pt[0], pt[1], pt[2]);
	    // Only insert a key if it has not been found previously, otherwise we repeat voxels
	    if(keys.find(key) == keys.end()) {
	      keys.insert(key);
	      ret.push_back({pt, tree->search(key)});
	    }
	  }

      return ret;
    }

    void Camera::pointAt(Point dir, double angle) {
      // Compute the transform corresponding to dir
      dir -= _loc.translation();

      // The z-axis is oriented in the Ray direction
      Eigen::Vector3f zAxis = dir.normalized();
      Eigen::Vector3f trans = _loc.translation();
      float invMag = 1.0 / zAxis.norm();
      // The y-axis is always 0 in the z direction
      Eigen::Vector3f yAxis(zAxis[1] * invMag, -zAxis[0] * invMag, 0);
      // The x-axis is the result of the cross product of the previous two
      Eigen::Vector3f xAxis = yAxis.cross(zAxis);

      // Keep position the same
      _loc.matrix() <<
	xAxis[0], yAxis[0], zAxis[0], trans[0],
	xAxis[1], yAxis[1], zAxis[1], trans[1],
	xAxis[2], yAxis[2], zAxis[2], trans[2],
	0,        0,        0,        1;
    }

    Camera::Intrinsics Camera::_convert(const CameraDescription &desc) {
      Camera::Intrinsics ret;

      ret <<
	desc.focalLength, 0,                desc.width / 2,
	0,                desc.focalLength, desc.height / 2,
	0,                0,                1;

      return ret;
    }
  } // active_vision
}   // scrap_burning
