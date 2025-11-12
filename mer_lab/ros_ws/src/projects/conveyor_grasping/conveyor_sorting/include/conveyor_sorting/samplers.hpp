#ifndef CONVEYOR_SORTING_SAMPLERS_HPP
#define CONVEYOR_SORTING_SAMPLERS_HPP

#include <random>
#include <vector>
#include <algorithm>

#include <qnd/geom.hpp>
#include <geometry_msgs/Point.h>

#include "types.hpp"

namespace conveyor_sorting {

    static uint32_t reverse32bit(uint32_t n) {
      n = (n << 16) | (n >> 16);
      n = ((n & 0x00ff00ff) << 8) | ((n & 0xff00ff00) >> 8);
      n = ((n & 0x0f0f0f0f) << 4) | ((n & 0xf0f0f0f0) >> 4);
      n = ((n & 0x33333333) << 2) | ((n & 0xcccccccc) >> 2);
      n = ((n & 0x55555555) << 1) | ((n & 0xaaaaaaaa) >> 1);
      return n;
    }

    static uint64_t reverse64bit(uint64_t n) {
      uint64_t n0 = reverse32bit((uint32_t)n);
      uint64_t n1 = reverse32bit((uint32_t)(n >> 32));
      return (n0 << 32) | n1;
    }

  class HammersleySampler {
  public:
    HammersleySampler(double height)
      : _height(height) {}

    double getHeight() const {return _height;}

    void setHeight(double height) {_height = height;}

    std::vector<geometry_msgs::Point> operator()(const cv::Mat& img,
						 const std::vector<Region>& regions,
						 std::size_t samples) {
      std::vector<geometry_msgs::Point> ret(samples);

      double scaleX{static_cast<double>(img.cols)};
      double scaleY{static_cast<double>(img.rows)};

      for(uint64_t i = 0; i < samples; ++i) {
	ret[i].x = static_cast<double>(i) / samples * scaleX;
	ret[i].y = reverse64bit(i) * 0x1p-64 * scaleY;
	ret[i].z = _height;
      }

      return ret;
    }
  private:
    double _height;
  };

  class ROIHammersleySampler {
  public:
    ROIHammersleySampler(double height, std::size_t x, std::size_t y,
                         std::size_t w, std::size_t l)
      : _height(height), _x(x), _y(y), _w(w), _l(l) {}

    double getHeight() const {return _height;}

    void setHeight(double height) {_height = height;}

    double getX() const {return _x;}
    double getY() const {return _y;}
    double getW() const {return _w;}
    double getL() const {return _l;}
    void setX(double x) {_x = x;}
    void setY(double y) {_y = y;}
    void setW(double w) {_w = w;}
    void setL(double l) {_l = l;}

    std::vector<geometry_msgs::Point> operator()(const cv::Mat& img,
						 const std::vector<Region>& regions,
						 std::size_t samples) {
      std::vector<geometry_msgs::Point> ret(samples);

      for(uint64_t i = 0; i < samples; ++i) {
        ret[i].x = static_cast<double>(i) / samples * _w + _x;
        ret[i].y = reverse64bit(i) * 0x1p-64 * _l + _y;
        ret[i].z = _height;
      }

      return ret;
    }
  private:
    double _height;
    double _x;
    double _y;
    double _w;
    double _l;
  };

  class AreaWeightedRandomSampler {
  public:
    AreaWeightedRandomSampler(std::size_t seed)
      : _seed(seed), _rng(seed) {}

    std::size_t getSeed() const {return _seed;}
    void setSeed(std::size_t seed) {_seed = seed; _rng.seed(seed);}

    std::vector<geometry_msgs::Point> operator()(const cv::Mat& img,
						 const std::vector<Region>& regions,
						 std::size_t samples) {
      std::vector<geometry_msgs::Point> ret(samples);

      std::vector<std::size_t> regionSizes(regions.size(), 0);
      std::transform(regions.begin(), regions.end(), regionSizes.begin(),
		     [](const Region& region) { return region.second.size(); });
      std::size_t totalRegionSize = std::accumulate(regionSizes.begin(), regionSizes.end(), 0);
      std::vector<int> pointsPerRegion(regions.size(), 0);
      std::transform(regionSizes.begin(), regionSizes.end(), pointsPerRegion.begin(),
		     [totalRegionSize, samples](std::size_t regionSize) {
		       return samples * (static_cast<double>(regionSize) / totalRegionSize);
		     });
      // With an array of points per region, we know how much to sample in each region
      std::size_t lastIdx{0};	// Index of last element insertion
      for(std::size_t i = 0; i < regions.size(); ++i) {
	std::vector<Point> regionPts(AreaWeightedRandomSampler::_randomSample(regions[i].second, pointsPerRegion[i]));

	std::transform(regionPts.begin(), regionPts.end(), ret.begin() + lastIdx,
		       [i, &regions](const Point& pt) {
			 return qnd::createPt(pt.first,
					      pt.second,
					      regions[i].first);
		       });
	lastIdx += regionPts.size();
      }

      // Remove unused values due to undersampling
      ret.erase(ret.begin() + lastIdx, ret.end());

      return ret;
    }
  private:
    std::size_t _seed;
    std::mt19937 _rng;

    template <typename T>
    std::vector<T> _randomSample(const std::vector<T>& vec, std::size_t count) {
      std::vector<T> ret(count);
      std::uniform_int_distribution<std::size_t> dist{0, vec.size() - 1};
      for(std::size_t i = 0; i < count; ++i)
	ret[i] = vec[dist(_rng)];

      return ret;
    }
  };

} // conveyor_sorting

#endif
