#ifndef CONVEYOR_SORTING_PROJECTOR_HPP
#define CONVEYOR_SORTING_PROJECTOR_HPP

#include <memory>
#include <string>

#include <qnd/tf.hpp>
#include <qnd/cam.hpp>
#include <qnd/geom.hpp>
#include <geometry_msgs/Point.h>

namespace conveyor_sorting {

  class OffsetProjector {
  public:
    OffsetProjector(double camOffsetHeight)
      : _camOffsetHeight(camOffsetHeight) {}

    double getCamOffsetHeight() const {return _camOffsetHeight;}
    void setCamOffsetHeight(double camOffsetHeight) {_camOffsetHeight = camOffsetHeight;}

    geometry_msgs::Point operator()(geometry_msgs::Point pt) {
      return pt;
    }
    std::vector<geometry_msgs::Point> operator()(std::vector<geometry_msgs::Point> pts) {
      std::transform(pts.begin(), pts.end(), pts.begin(),
		     [this](const geometry_msgs::Point& pt) {return this->operator()(pt);});
      return pts;
    }
    geometry_msgs::Point deproject(geometry_msgs::Point pt) {
      return pt;
    }

    std::string getTargetFrame() const {return "";}
  private:
    double _camOffsetHeight;
  };

  class TopographicalProjector {
  public:
    TopographicalProjector(std::shared_ptr<qnd::RosCam> sourceCam, double camOffsetHeight, const std::string& targetFrame="world")
      : _sourceCam(sourceCam), _camOffsetHeight(camOffsetHeight), _targetFrame(targetFrame) {}

    std::string getTargetFrame() const {return _targetFrame;}
    void setTargetFrame(const std::string& targetFrame) {_targetFrame = targetFrame;}

    double getCamOffsetHeight() const {return _camOffsetHeight;}
    void setCamOffsetHeight(double camOffsetHeight) {_camOffsetHeight = camOffsetHeight;}

    geometry_msgs::Point operator()(geometry_msgs::Point pt) {
      // Convert the point from topographical map coordinates into camera frame coordinates
      pt = qnd::createPt(pt.y, pt.x, _camOffsetHeight - pt.z);
      // Project the point to the camera frame
      pt = _sourceCam->fromImgCoord(pt);
      // Transform it into the world frame based on the last-received camera frame
      return *qnd::transform(pt, _targetFrame, _sourceCam->getLatestImgFrame());
    };
    geometry_msgs::Point deproject(geometry_msgs::Point pt) {
      // Reverse the above steps
      pt = *qnd::transform(pt, _sourceCam->getLatestImgFrame(), _targetFrame);
      pt = _sourceCam->toImgCoord(pt);
      return qnd::createPt(pt.x, pt.y, _camOffsetHeight - pt.z);
    }
    std::vector<geometry_msgs::Point> operator()(std::vector<geometry_msgs::Point> pts) {
      std::transform(pts.begin(), pts.end(), pts.begin(),
		     [this](const geometry_msgs::Point& pt) {return this->operator()(pt);});
      return pts;
    }
  private:
    std::shared_ptr<qnd::RosCam> _sourceCam;
    double _camOffsetHeight;
    std::string _targetFrame;
  };

} // conveyor_sorting

#endif
