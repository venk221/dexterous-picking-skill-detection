#ifndef CONVEYOR_SORTING_IMG_HPP
#define CONVEYOR_SORTING_IMG_HPP

#include <vector>

#include <opencv2/opencv.hpp>

#include "types.hpp"

namespace conveyor_sorting {

  // Helper functions
  std::vector<Point> filterIndices(const cv::Mat& img, ushort val) {
    std::vector<Point> ret;

    for(std::size_t i = 0; i < img.rows; ++i)
      for(std::size_t j = 0; j < img.cols; ++j)
	if(img.at<ushort>(i, j) == val)
	  ret.push_back({j, i});

    return ret;
  }

  // The maximum float value an image can have
  static constexpr float maxFloat{1.0};

  template <typename MapSampler, typename TargetSampler>
  class ImgProcessor {
  public:
    ImgProcessor(MapSampler mSampler, TargetSampler tSampler,
		 const std::vector<double>& heights, std::size_t samples)
      : _mSampler(mSampler), _tSampler(tSampler), _heights(heights), _samples(samples) {}

    MapSampler getMapSampler() const {return _mSampler;}
    TargetSampler getTargetSampler() const {return _tSampler;}
    const std::vector<double>& getHeights() const {return _heights;}
    std::size_t getSamples() const {return _samples;}

    void setMapSampler(MapSampler mSampler) {_mSampler = mSampler;}
    void setTargetSampler(TargetSampler tSampler) {_tSampler = tSampler;}
    void setHeights(std::vector<double> heights) {_heights.swap(heights);}
    void setSamples(std::size_t samples) {_samples = samples;}

    UnassociatedPoints operator()(const cv::Mat& img) {
      ROS_DEBUG("Processing image");

      std::vector<Region> regions(_splitMat(img));
      ROS_DEBUG_STREAM("Split map into " << regions.size() << " regions");
      std::vector<geometry_msgs::Point> mapPts(_mSampler.operator()(img, regions, _samples));
      ROS_DEBUG_STREAM("Sampled " << mapPts.size() << " points on the topographical map");
      // We generate mapPts.size() tgtPts, since it may not be _samples in size exactly
      std::vector<geometry_msgs::Point> tgtPts(_tSampler.operator()(img, regions, mapPts.size()));
      ROS_DEBUG_STREAM("Sampled " << tgtPts.size() << " points as targets");

      return {mapPts, tgtPts};
    }
  private:
    MapSampler _mSampler;
    TargetSampler _tSampler;
    std::vector<double> _heights;
    std::size_t _samples;

    std::vector<Region> _splitMat(const cv::Mat& img) {
      std::vector<Region> ret{};

      for(double height : _heights) {
	cv::Mat tmpThresh{};
	cv::threshold(img, tmpThresh, height, maxFloat, 0);

	// Convert into specific format to run cv::connectedComponents
	cv::Mat threshOut{};
	tmpThresh.convertTo(threshOut, CV_8U, 255, 0);

	cv::Mat islands{};
	// Connectivity is 8
	int connectedComponents{cv::connectedComponents(threshOut, islands, 8, CV_16U)};
	for(int i = 1; i < connectedComponents; ++i)
	  ret.push_back(Region{height, filterIndices(islands, i)});

	// cv::Mat cIslands;
	// islands.convertTo(cIslands, CV_8U, 255 / connectedComponents);
	// cv::Mat coloredIslands;
	// cv::applyColorMap(cIslands, coloredIslands, cv::COLORMAP_JET);
	// cv::imwrite(std::string("/tmp/") + std::to_string(height) + ".png", coloredIslands);
      }

      return ret;
    }
  };

} // conveyor_sorting

#endif
