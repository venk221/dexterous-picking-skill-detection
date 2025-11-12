#ifndef SCRAP_BURNING_ACTIVE_VISION_CAMERA_HPP
#define SCRAP_BURNING_ACTIVE_VISION_CAMERA_HPP

#include <Eigen/Geometry>

#include "scrap_burning/active_vision/core.hpp"
#include "scrap_burning/active_vision/grid.hpp"

namespace scrap_burning {
  namespace active_vision {

    // Camera class, contains methods that facilitate active vision
    class Camera {
    public:
      // The dimension used for width and height, in pixels
      using Dim = uint16_t;

      // struct representing camera internal description, required to properly project to/from image-space
      struct CameraDescription {
	Dim width;
	Dim height;
	double focalLength;
      };

      // A Camera object requires a description of its internals, and a transform representing position and orientation
      // The z-axis in the camera transform is the direction the camera is facing
      // The y-axis in the camera transform is the upwards direction in the camera space
      // The x-axis in the camera transform is the left direction in the camera space

      // This is the default camera direction if none are specified
      Camera(const CameraDescription &desc, const Transform &loc=Transform::Identity());

      // Get the camera description
      CameraDescription getDescription() const {return _desc;}

      // Return the position of the camera discretized on the octomap
      octomap::OcTreeKey getDiscretePosition(const Map *tree) const;

      // Checks if a point is within this camera's view
      bool isWithin(const Point &p) const;

      // Casts a ray from the camera center towards a pixel
      Ray cast(Dim x, Dim y) const;

      // Returns a list of all cells that are within this camera's view
      RayCells getViewedCells(const Map *tree, double l=1.0);

      Transform getTransform() const {return _loc;}
      void setTransform(const Transform &tf) {_loc = tf;}

      void pointAt(Point dir, double angle=0.0);
    private:
      // Transform used to denote an intrinsics matrix
      typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> Intrinsics;

      // Camera internal details
      CameraDescription _desc;

      // Camera location in space
      Transform _loc;

      // Static helper method that converts a CameraDescription to an intrinsics matrix for projection
      static Intrinsics _convert(const CameraDescription &desc);
    };

  };				// active_vision
};				// scrap_burning

#endif	// SCRAP_BURNING_ACTIVE_VISION_CAMERA_HPP
