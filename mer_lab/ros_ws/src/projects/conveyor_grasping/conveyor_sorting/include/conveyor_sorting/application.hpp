#ifndef CONVEYOR_SORTING_APPLICATION_HPP
#define CONVEYOR_SORTING_APPLICATION_HPP

#include <vector>

#include <ros/ros.h>
#include <qnd/vis.hpp>
#include <conveyor_sorting_msgs/SweepingAction.h>

#include <opencv2/opencv.hpp>

#include "types.hpp"

namespace conveyor_sorting {

  template <typename ImgProcessor, typename CostCalculator, typename Sweeper,
	    typename OTCalculator, typename Projector>
  class Application {
  public:
    Application(ImgProcessor processor, CostCalculator costCalculator, Sweeper sweeper,
		OTCalculator otCalculator, Projector proj, qnd::Visualizer& vis, bool visualizeIntermediate=true)
      : _processor(processor), _costCalculator(costCalculator), _sweeper(sweeper),
	_otCalculator(otCalculator), _proj(proj), _visualizeIntermediate(visualizeIntermediate),
	_vis(vis) {}

    ImgProcessor getImgProcessor() const {return _processor;}
    void setImgProcessor(ImgProcessor processor) {_processor = processor;}

    CostCalculator getCostCalculator() const {return _costCalculator;}
    void setCostCalculator(CostCalculator costCalculator) {_costCalculator = costCalculator;}

    Sweeper getSweeper() const {return _sweeper;}
    void setSweeper(Sweeper sweeper) {_sweeper = sweeper;}

    OTCalculator getOTCalculator() const {return _otCalculator;}
    void setOTCalculator(OTCalculator otCalculator) {_otCalculator = otCalculator;}

    Projector getProjector() const {return _proj;}
    void setProjector(Projector proj) {_proj = proj;}

    bool getVisualizeIntermediate() const {return _visualizeIntermediate;}
    void setVisualizeIntermediate(bool visualizeIntermediate) {_visualizeIntermediate = visualizeIntermediate;}

    conveyor_sorting_msgs::SweepingAction operator()(const cv::Mat& rgbImg, const cv::Mat& topImg) {
      // Sample a set of points along the topographical map and the target
      UnassociatedPoints pts{_processor(topImg)};
      if(_visualizeIntermediate) {
	_vis.visPoint(0, _proj(pts.first), _proj.getTargetFrame(), qnd::createPt(0.01), 1.0, 0.0, 0.0);
	_vis.visPoint(1, _proj(pts.second), _proj.getTargetFrame(), qnd::createPt(0.01), 0.0, 1.0, 0.0);
      }

      // If we have no points at all, that means one of two things:
      //  1. An error sampling points
      //  2. No topographical map was produced
      //  3. All points have been swept out of the region of interest
      // Either way, we need to stop here, no sense moving forward
      if(pts.first.size() == 0 || pts.second.size() == 0) {
	ROS_WARN("No points sampled on the map or target, skipping processing");
	conveyor_sorting_msgs::SweepingAction ret{};
	ret.start = qnd::createPt(0);
	ret.end = qnd::createPt(0);
	return ret;
      }

      // Compute the cost
      std::vector<double> cost{_costCalculator(pts)};

      // Solve the optimal transport problem and associate the points
      AssociatedPoints associatedPts{_otCalculator(pts, cost)};

      // Compute a sweeping action from the final associated points
      conveyor_sorting_msgs::SweepingAction action{_sweeper(associatedPts)};
      // if(_visualizeIntermediate) {
      // 	_vis.visArrow(2, {_proj(action.start), _proj(action.end)}, _proj.getTargetFrame(),
      // 		      qnd::createPt(0.01, 0.01, 0.0));
      // }

      // All done, return the final action, converted into world coordinates
      action.start = _proj(action.start);
      action.end = _proj(action.end);
      return action;
    }
  private:
    // Functors
    ImgProcessor _processor;
    CostCalculator _costCalculator;
    Sweeper _sweeper;
    OTCalculator _otCalculator;
    Projector _proj;
    qnd::Visualizer& _vis;

    bool _visualizeIntermediate;

    // Services
    ros::ServiceClient otClient;
  };

} // conveyor_sorting

#endif
