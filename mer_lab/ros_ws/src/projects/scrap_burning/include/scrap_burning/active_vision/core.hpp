#ifndef SCRAP_BURNING_ACTIVE_VISION_RAY_HPP
#define SCRAP_BURNING_ACTIVE_VISION_RAY_HPP

#include <cmath>
#include <iostream>

#include "octomap/ColorOcTree.h"

// Eigen includes
#include <Eigen/Dense>

namespace scrap_burning {
  namespace active_vision {
    // The type of octomap and node used
    typedef octomap::ColorOcTree Map;
    typedef octomap::ColorOcTreeNode Node;

    // The type used to represent transforms
    typedef Eigen::Transform<float, 3, Eigen::Isometry, Eigen::DontAlign> Transform;

    typedef Eigen::Vector3f Point;

    class OctoRay {
    public:
      OctoRay(const octomap::point3d &from, const octomap::point3d &to)
	: _origin(from), _vector((to - _origin).normalized()) {}

      const octomap::point3d &getOrigin() const {return _origin;}
      const octomap::point3d &getVector() const {return _vector;}

      octomap::point3d &getOrigin() {return _origin;}
      octomap::point3d &getVector() {return _vector;}

      // Walks along the Ray by a distance t from _origin
      octomap::point3d operator()(float t) const {
	return _origin + _vector * t;
      }
    private:
      octomap::point3d _origin;
      octomap::point3d _vector;
    };

    // Represents a Ray, moves infinitely in a given direction
    class Ray {
    public:
      // Only one constructor for now
      Ray(const Point &from, const Point &to)
	: _origin(from), _vector((to - _origin).normalized()) {}

      const Point &getOrigin() const {return _origin;}
      const Point &getVector() const {return _vector;}
      Point &getOrigin() {return _origin;}
      Point &getVector() {return _vector;}

      // Walks along the Ray by a distance t from _origin
      Point operator()(float t) const {
	return _origin + _vector * t;
      }

      // Transformation operator
      Ray &operator*=(const Transform &tf) {
	// Convert the _vector into an eigen datatype
	_vector = tf.rotation() * _vector;
	_origin = tf * _origin;

	return *this;
      }
    private:
      // Internally, a Ray is represented by an _origin and a _vector
      // _vector is guaranteed to be a unit vector
      Point _origin;
      Point _vector;
    };

    // ostream operator
    inline std::ostream &operator<<(std::ostream &os, const Ray &ray) {
      os << ray.getOrigin() << " -> " << ray.getVector();

      return os;
    }

    // Helper functions
    inline int getNearest(const Point &src, const std::vector<Point> &tgts) {
      double minDist = 10000;
      int ret = -1;

      for(int i = 0; i < tgts.size(); ++i) {
	const auto &tgt = tgts[i];
	double dist = (src - tgt).norm();
	if(dist < minDist) {
	  minDist = dist;
	  ret = i;
	}
      }

      return ret;
    }

    inline Point computeCentroid(const std::vector<Point> &cells) {
      Point pt(0, 0, 0);

      for(const auto &cell : cells) pt += cell;
      pt /= cells.size();

      return pt;
    }

    class Cluster {
    public:
      Cluster() : _pts() {}
      Cluster(const std::vector<Point> &pts) : _pts(pts) {}

      Point getCentroid() const {
	return computeCentroid(_pts);
      }

      std::vector<Point> &getPts() { return _pts; }
      const std::vector<Point> &getPts() const { return _pts; }
    private:
      std::vector<Point> _pts;
    };

    // We store a set of voxels here
    typedef std::pair<scrap_burning::active_vision::Point, Node*> PositionedCell;
    typedef std::vector<PositionedCell> RayCells;

  } // active_vision
}   // scrap_burning

#endif	// SCRAP_BURNING_ACTIVE_VISION_RAY_HPP
