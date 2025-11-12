#ifndef CONVEYOR_SORTING_SWEEPING_HPP
#define CONVEYOR_SORTING_SWEEPING_HPP

#include <vector>
#include <numeric>
#include <algorithm>
#include <unordered_set>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <qnd/geom.hpp>
#include <qnd/conv.hpp>

#include <conveyor_sorting_msgs/SweepingAction.h>

#include "types.hpp"

namespace conveyor_sorting {

  // Helper functions
  geometry_msgs::Point createUnitVector(double angle) {
    geometry_msgs::Point ret;

    ret.x = std::cos(angle);
    ret.y = std::sin(angle);
    ret.z = 0.0;

    return ret;
  }
  // Compute the centroid of the source distribution
  geometry_msgs::Point sourceCentroid(const AssociatedPoints& map, double height) {
    geometry_msgs::Point ret{qnd::multiply(std::accumulate(map.begin(), map.end(), qnd::createPt(0, 0, 0),
							   [](const geometry_msgs::Point& pt, const PointPair& ptPair) {
					   return qnd::add(pt, ptPair.first);
					 }),
					   1.0 / static_cast<double>(map.size()))};
    ret.z = height;

    return ret;
  }

  geometry_msgs::Point getIntersection(const geometry_msgs::Point& lineA1, const geometry_msgs::Point& lineA2,
                                       const geometry_msgs::Point& lineB1, const geometry_msgs::Point& lineB2) {
    // Compute intersection
    // Line AB represented as a1x + b1y = c1
    double a1 = lineA2.y - lineA1.y;
    double b1 = lineA1.x - lineA2.x;
    double c1 = a1*(lineA1.x) + b1*(lineA1.y);
  
    // Line CD represented as a2x + b2y = c2
    double a2 = lineB2.y - lineB1.y;
    double b2 = lineB1.x - lineB2.x;
    double c2 = a2*(lineB1.x)+ b2*(lineB1.y);
  
    double determinant = a1*b2 - a2*b1;
  
    double x = (b2*c1 - b1*c2)/determinant;
    double y = (a1*c2 - a2*c1)/determinant;
    return qnd::createPt(x, y, lineA1.z);
  }

  PointPair expandTillBorder(const geometry_msgs::Point& pt, const geometry_msgs::Point& dir,
			     std::size_t w, std::size_t h) {
    ROS_WARN_STREAM(qnd::magnitude(dir));
    double mag{std::sqrt(w * w + h * h)};
    // double fwdXBound{pt.x};
    // double bckXBound{-pt.x};
    // if(dir.x > 0.0)
    //   fwdXBound = w - pt.x;
    // else
    //   bckXBound = -w + pt.x;
    // double fwdYBound{pt.y};
    // double bckYBound{-pt.y};
    // if(dir.y > 0.0)
    //   fwdYBound = h - pt.y;
    // else
    //   bckYBound = -h + pt.y;

    // double fwdXDist = fwdXBound / dir.x;
    // double fwdYDist = fwdYBound / dir.y;
    // double bckXDist = fwdXBound / dir.x;
    // double bckYDist = fwdYBound / dir.y;
    // double fwdMag = (fwdXDist > fwdYDist ? fwdYDist : fwdXDist);
    // double bckMag = (bckXDist > bckYDist ? bckYDist : bckXDist);

    // return {qnd::add(pt, qnd::multiply(dir, -bckMag)),
    // 	    qnd::add(pt, qnd::multiply(dir, fwdMag))};
    return {qnd::add(pt, qnd::multiply(dir, -mag / 2)),
	    qnd::add(pt, qnd::multiply(dir, mag / 2))};
  }

  PointPair expandTillBorder(const geometry_msgs::Point& pt, const geometry_msgs::Point& dir,
                             const AssociatedPoints& ptMap) {
    // Compute the borders of the ptMap
    double minX{1000};
    double maxX{-minX};
    double minY{minX};
    double maxY{maxX};
    for(const auto& ptPair : ptMap) {
      if(ptPair.second.x < minX) minX = ptPair.second.x;
      if(ptPair.second.x > maxX) maxX = ptPair.second.x;
      if(ptPair.second.y < minY) minY = ptPair.second.y;
      if(ptPair.second.y > maxY) maxY = ptPair.second.y;
    }
    // Expand till we hit those borders
    // Vertical line
    if(dir.x == 0.0) {
      // Determine where we will begin in y
      double startY = minY;
      double endY = maxY;
      if(dir.y < 0) {
        startY = maxY;
        endY = minY;
      }
      return PointPair{qnd::createPt(pt.x, startY, pt.z), qnd::createPt(pt.x, endY, pt.z)};
    }
    // Horizontal line
    if(dir.y == 0.0) {
      // Determine where we will begin in y
      double startX = minX;
      double endX = maxX;
      if(dir.x < 0) {
        startX = maxX;
        endX = minX;
      }
      return PointPair{qnd::createPt(startX, pt.y, pt.z), qnd::createPt(endX, pt.y, pt.z)};
    }
    // General case
    // If the direction vector is positive in x, then the rightmost wall is the only one we will hit in x
    // If the direction vector is positive in y, then we need to check the rightmost and topmost wall
    PointPair ret{};
    geometry_msgs::Point bckDir(qnd::multiply(dir, -1.0));
    geometry_msgs::Point xLimit(getIntersection(pt, qnd::add(pt, dir),
                                                qnd::createPt(maxX, minY, pt.z), qnd::createPt(maxX, maxY, pt.z)));
    geometry_msgs::Point bckXLimit(getIntersection(pt, qnd::add(pt, bckDir),
                                                  qnd::createPt(minX, minY, pt.z), qnd::createPt(minX, maxY, pt.z)));
    if(dir.x < 0.0) {
      // xLimit = getIntersection(pt, qnd::add(pt, dir), qnd::createPt(minX, minY, pt.z, qnd::createPt(minX, maxY, pt.z)));
      std::swap(xLimit, bckXLimit);
    }
    if(xLimit.y > maxY)
      ret.second = getIntersection(pt, qnd::add(pt, dir), qnd::createPt(minX, maxY, pt.z), qnd::createPt(maxX, maxY, pt.z));
    else if(xLimit.y < minY)
      ret.second = getIntersection(pt, qnd::add(pt, dir), qnd::createPt(minX, minY, pt.z), qnd::createPt(maxX, minY, pt.z));
    else
      ret.second = xLimit;

    if(bckXLimit.y > maxY)
      ret.first = getIntersection(pt, qnd::add(pt, bckDir), qnd::createPt(minX, maxY, pt.z), qnd::createPt(maxX, maxY, pt.z));
    else if(bckXLimit.y < minY)
      ret.first = getIntersection(pt, qnd::add(pt, bckDir), qnd::createPt(minX, minY, pt.z), qnd::createPt(maxX, minY, pt.z));
    else
      ret.first = bckXLimit;

    return ret;
  }

  static constexpr double pi{3.141592653589};

  class DotProductSweeper {
  public:
    DotProductSweeper(std::size_t angleCount, const std::vector<double>& topMapHeights, Size imgSize)
      : _angleCount{angleCount}, _topMapHeights(topMapHeights), _imgSize{imgSize} {}

    std::size_t getAngleCount() const {return _angleCount;}
    const std::vector<double>& getTopMapHeights() const {return _topMapHeights;}
    Size getImageSize() const {return _imgSize;}

    void setAngleCount(std::size_t angleCount) {_angleCount = angleCount;}
    void setTopMapHeights(std::vector<double> topMapHeights) {_topMapHeights.swap(topMapHeights);}
    void setImgSize(Size imgSize) {_imgSize.swap(imgSize);}

    conveyor_sorting_msgs::SweepingAction operator()(const AssociatedPoints& map) {
      double bestHeight{0.0};
      geometry_msgs::Point bestVector{};
      double bestValue{-1.0};	// Worst possible value is -1.0 since we normalize

      std::vector<geometry_msgs::Point> unitVectors{};
      unitVectors.reserve(_angleCount);
      for(double angle{0.0}; angle < pi; angle += (pi / _angleCount))
      	unitVectors.push_back(createUnitVector(angle));

      for(double height : _topMapHeights) {
        std::pair<geometry_msgs::Point, double> result(_getBestAngle(map, height));
        if(result.second > bestValue) {
          bestHeight = height;
          bestVector = result.first;
          bestValue = result.second;
        }
      }

      // After the above loop, we should have the best height, sweeping vector direction, and value
      // Our goal now is to convert those into a SweepingAction
      // We can assume the centroid of the pile is the centroid of the source points
      geometry_msgs::Point centroid{sourceCentroid(map, bestHeight)};
      // Expand till we hit the borders of the image
      PointPair expandedDir(expandTillBorder(centroid, bestVector, map));
      conveyor_sorting_msgs::SweepingAction ret{};
      ret.start = expandedDir.first;
      ret.end = expandedDir.second;
      return ret;
    }
  private:
    std::size_t _angleCount;
    std::vector<double> _topMapHeights;
    Size _imgSize;

    // Returns vector - score
    std::pair<geometry_msgs::Point, double> _getBestAngle(AssociatedPoints map, double height) {
      std::pair<geometry_msgs::Point, double> ret{qnd::createPt(0.0), -1.0};
      // Filter out all points below this level
      map.erase(std::remove_if(map.begin(), map.end(),
			       [height](const PointPair& ptPair) {
				 return ptPair.first.z < height;
			       }), map.end());

      // Early termination if no points were found
      if(map.size() == 0)
	return ret;

      // Remove the z
      std::transform(map.begin(), map.end(), map.begin(),
		     [](const PointPair& ptPair) {
		       return PointPair{qnd::createPt(ptPair.first.x,
						      ptPair.first.y,
						      0.0),
					  qnd::createPt(ptPair.second.x,
							ptPair.second.y,
							0.0)};
		     });
      // Compute the dot product for each angle
      for(double angle{0.0}; angle < pi; angle += (pi / _angleCount)) {
	geometry_msgs::Point sweepingVector{createUnitVector(angle)};
	std::vector<double> dotProds(map.size());
	std::transform(map.begin(), map.end(), dotProds.begin(),
		       [&sweepingVector](const PointPair& ptPair) {
			 return qnd::dotProd(sweepingVector, qnd::normalize(qnd::sub(ptPair.second, ptPair.first)));
		       });
	double normDotProd{std::accumulate(dotProds.begin(), dotProds.end(), 0.0) / dotProds.size()};
	// Assign if it is better than the best so far
	if(normDotProd > ret.second) {
	  ret.first = sweepingVector;
	  ret.second = normDotProd;
	}
      }
      return ret;
    }

  };

  class CSweeper {
  public:
    CSweeper(std::size_t angleCount, const std::vector<double>& topMapHeights, const geometry_msgs::Point& size)
      : _angleCount(angleCount), _topMapHeights(topMapHeights), _size(size) {}

    std::size_t getAngleCount() const {return _angleCount;}
    const std::vector<double>& getTopMapHeights() const {return _topMapHeights;}
    geometry_msgs::Point getSize() const {return _size;}

    void setAngleCount(std::size_t angleCount) {_angleCount = angleCount;}
    void setTopMapHeights(std::vector<double> topMapHeights) {_topMapHeights.swap(topMapHeights);}
    void setSize(const geometry_msgs::Point& size) {_size = size;}

    conveyor_sorting_msgs::SweepingAction operator()(const AssociatedPoints& map) {
      conveyor_sorting_msgs::SweepingAction ret{};

      double bestScore{-10000};
      for(double height : _topMapHeights) {
	std::pair<conveyor_sorting_msgs::SweepingAction, double> result(_getBestSweep(map, height));
	if(result.second > bestScore) {
	  bestScore = result.second;
	  ret = result.first;
	}
      }

      return ret;
    }
  private:
    std::size_t _angleCount;
    std::vector<double> _topMapHeights;
    geometry_msgs::Point _size;

    std::pair<conveyor_sorting_msgs::SweepingAction, double> _getBestSweep(const AssociatedPoints& map, double height) {
      std::pair<conveyor_sorting_msgs::SweepingAction, double> ret{};

      // Best score is 0 since if we get something even worse, just stop here
      double bestScore{0.0};
      // We need to iterate here over all angles, and for each angle, over offsets
      for(double angle{0.0}; angle < pi; angle += (pi / _angleCount)) {
	std::pair<conveyor_sorting_msgs::SweepingAction, double> result(_getBestOffset(map, height, angle));
	if(result.second > bestScore) {
	  bestScore = result.second;
	  ret = result;
	}
      }

      return ret;
    }

    std::pair<conveyor_sorting_msgs::SweepingAction, double> _getBestOffset(AssociatedPoints map, double height, double angle) {
      std::pair<conveyor_sorting_msgs::SweepingAction, double> ret{};

      geometry_msgs::Point unitVec(createUnitVector(angle));

      auto printMap = [](const AssociatedPoints& map) {
	for(const auto& ptPair : map) {
	  std::cout << qnd::toString(ptPair.first) << " -> " << qnd::toString(ptPair.second) << '\n';
	}
      };

      // std::cout << "Started out with:\n";
      // printMap(map);

      // Filter out all points in the associated map where the z of the source is not height
      map.erase(std::remove_if(map.begin(), map.end(), [height](const PointPair& pt) {
	return pt.first.z != height;
      }), map.end());

      // std::cout << "Filtering by height " << height << '\n';
      if(map.size() == 0)	// No points, stop early
	return ret;

      // printMap(map);

      // Strip the z
      std::transform(map.begin(), map.end(), map.begin(),
		     [](const PointPair& ptPair) {
		       return PointPair{qnd::createPt(ptPair.first.x,
						      ptPair.first.y,
						      0.0),
			 qnd::createPt(ptPair.second.x,
				       ptPair.second.y,
				       0.0)};
		     });
      
      // Start moving and decide on an offset
      // To start, compute a rotation matrix, and rotate all points to align the axes
      // We know the y axis is aligned with the sweeping direction
      geometry_msgs::Point xAxis(qnd::crossProd(unitVec, qnd::createPt(0, 0, 1)));
      Eigen::Matrix3d rot;
      rot <<
	xAxis.x, unitVec.x, 0,
	xAxis.y, unitVec.y, 0,
	xAxis.z, unitVec.z, 1;
      Eigen::Matrix3d rott = rot.transpose();

      // Multiple each point by the above rotation matrix
      std::transform(map.begin(), map.end(), map.begin(), [&rott](const PointPair& pt) {
	return PointPair{qnd::eigenToGeom(rott * qnd::geomToEigen(pt.first)),
	  qnd::eigenToGeom(rott * qnd::geomToEigen(pt.second))};
      });
      // std::cout << "Rotating all points by " << rott << '\n';
      // printMap(map);

      // At this point, we have a bunch of rotated points, it should be easy to start iterating
      // Sort points by order of increasing x, so left -> right
      std::sort(map.begin(), map.end(), [](const PointPair& pt_a, const PointPair& pt_b) {
	return pt_a.first.x < pt_b.first.x;
      });
      // std::cout << "Sorting:\n";
      // printMap(map);

      // Once sorted, we can start going from the start to the end
      // The left-most y is at the minimum point - width / 2
      // The right-most y is at the maximum point + width / 2
      // We step by a given distance, say 1/2 cm
      static constexpr double stepSize{0.05};
      // Store the top point and offset here
      double bestStart{0.0};
      double bestLengthS{0.0};
      double bestLengthE{0.0};
      double bestScore{-10000.0};
      for(double start=map[0].first.x - _size.x / 2; start <= map.back().first.x + _size.x / 2;
	  start += stepSize) {
	// Check which points are within the push
	std::vector<std::size_t> includedPts{};
	auto startRange{std::upper_bound(map.begin(), map.end(), start - _size.x / 2, [](double lim, const PointPair& pt) {
	  return lim < pt.first.x;
	})};
	auto endRange{std::upper_bound(startRange, map.end(), start + _size.x / 2, [](double lim, const PointPair& pt) {
	  return lim < pt.first.x;
	})};
	// Out of bounds
	if(startRange == map.end() || endRange == map.begin())
	  continue;

	// Compute the best score across the length
	std::tuple<double, double, double> lsPair = _getBestLength(map);
	if(std::get<2>(lsPair) > bestScore) {
	  bestLengthS = std::get<0>(lsPair);
	  bestLengthE = std::get<1>(lsPair);
	  bestScore = std::get<2>(lsPair);
	  bestStart = start;
	}
      }
      // Move backwards, so start from there
      geometry_msgs::Point start;
      geometry_msgs::Point end;
      start = qnd::createPt(bestStart, bestLengthS, 0.0);
      end = qnd::createPt(bestStart, bestLengthE, 0.0);
      // std::cout << "This means we go from " << qnd::toString(start) << " -> " << qnd::toString(end) << '\n';
      ret.first.start = qnd::eigenToGeom(rot * qnd::geomToEigen(start));
      ret.first.end = qnd::eigenToGeom(rot * qnd::geomToEigen(end));
      // Eigen::Vector3d tfCoord = qnd::geomToEigen(qnd::createPt(bestStart, 0.0, 0.0));
      // ret.first = qnd::eigenToGeom(rot * tfCoord);
      ret.first.start.z = height;
      ret.first.end.z = height;
      ret.second = bestScore;
      return ret;
    }

    std::tuple<double, double, double> _getBestLength(AssociatedPoints map) {
      auto printMap = [](const AssociatedPoints& map) {
	for(const auto& ptPair : map) {
	  std::cout << qnd::toString(ptPair.first) << " -> " << qnd::toString(ptPair.second) << '\n';
	}
      };
      
      // The minimum sweep is at the smallest y value in the map
      std::sort(map.begin(), map.end(), [](const PointPair& a, const PointPair& b) {
	return a.first.y < b.first.y;
      });
      // Start looping across each point and add it to the list
      double bestDotProd{-10000};
      double bestStart{0.0};
      double bestEnd{0.0};
      for(std::size_t j = 0; j < map.size(); ++j) {
	double startY = map[j].first.y;
	for(std::size_t i = 0; i < map.size(); ++i) {
	  if(i == j) continue;
	  double endY = map[i].first.y;
	  geometry_msgs::Point sweepVec = qnd::createPt(0, endY - startY, 0);
	  // Compute the dot product
	  std::size_t start = i;
	  std::size_t end = j;
	  if(j < i) {
	    start = j;
	    end = i;
	  };
	  double cumDotProd = std::accumulate(map.begin() + start, map.begin() + end, 0.0,
					      [&sweepVec](double val, const PointPair& a) {
						return val + qnd::dotProd(sweepVec,
									  qnd::sub(a.second, a.first));
					      });
	  // Check how good it is
	  if(cumDotProd > bestDotProd) {
	    bestStart = startY;
	    bestEnd = endY;
	    bestDotProd = cumDotProd;
	  }
	}
      }

      // All done, return the best one found
      return {bestStart, bestEnd, bestDotProd};
    }
  };

  class RandomSweeper {
  public:
    RandomSweeper(std::size_t angleCount, const std::vector<double>& topMapHeights, Size imgSize,
		  std::size_t seed)
      : _angleCount{angleCount}, _topMapHeights(topMapHeights), _imgSize(imgSize), _angles{},
	_seed(seed), _rng(seed) {
	  // Update the angle count
	  _updateAngles();
	}

    std::size_t getAngleCount() const {return _angleCount;}
    const std::vector<double>& getTopMapHeights() const {
      for(std::size_t i = 0; i < _topMapHeights.size(); ++i) {
      }
      return _topMapHeights;
    }
    Size getImageSize() const {return _imgSize;}
    std::size_t getSeed() const {return _seed;}

    void setAngleCount(std::size_t angleCount) {_angleCount = angleCount; _updateAngles();}
    void setTopMapHeights(std::vector<double> topMapHeights) {_topMapHeights.swap(topMapHeights);}
    void setImgSize(Size imgSize) {_imgSize.swap(imgSize);}
    void setSeed(std::size_t seed) {_seed = seed; _rng.seed(seed);}

    conveyor_sorting_msgs::SweepingAction operator()(const AssociatedPoints& map) {
      // TODO: Implement
      // Pick a random height as long as there is at least one point there
      std::unordered_set<double> mapHeights{};
      // std::cout << map.size() << '\n';
      for(const PointPair& mapPair : map) {
	mapHeights.insert(mapPair.first.z);
      }
      double height{};
      if(mapHeights.size() == 1) {
	std::cout << "Found a map of height 1\n";
	height = *mapHeights.begin();
      }
      else if(mapHeights.empty()) {
	std::cout << "Empty map found!  Impossible!\n";
	height = *mapHeights.begin();
      }
      else
	double height{_getRandom(std::vector<double>(mapHeights.begin(), mapHeights.end()))};
      // Pick a random angle
      double angle{_getRandom(_angles)};

      // Compute the centroid
      geometry_msgs::Point centroid{sourceCentroid(map, height)};
      // Expand to fill the image
      // First, make a unit vector in the angle direction
      PointPair expandedDir(expandTillBorder(centroid, createUnitVector(angle), map));

      conveyor_sorting_msgs::SweepingAction ret{};
      ret.start = expandedDir.first;
      ret.end = expandedDir.second;
      return ret;
    }
  private:
    std::size_t _angleCount;
    std::vector<double> _topMapHeights;
    Size _imgSize;
    std::vector<double> _angles;
    std::size_t _seed;
    std::mt19937 _rng;

    void _updateAngles() {
      std::vector<double> newAngles{};
      newAngles.reserve(_angleCount);

      for(std::size_t i = 0; i < _angleCount; ++i) {
	newAngles.push_back((static_cast<double>(i) / (_angleCount - 1)) * 2 * pi);
      }

      _angles.swap(newAngles);
    }

    template <typename T>
    T _getRandom(const std::vector<T>& vec) {
      std::uniform_int_distribution<std::size_t> elementDist(0, vec.size() - 1);
      std::size_t idx{elementDist(_rng)};
      return vec[idx];
    }
  };

} // conveyor_sorting

#endif
