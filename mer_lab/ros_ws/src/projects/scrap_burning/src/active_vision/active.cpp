#include "scrap_burning/active_vision/active.hpp"

using scrap_burning::active_vision::OctoRay;
using scrap_burning::active_vision::Map;
using scrap_burning::active_vision::KeyMap;
using Dim = scrap_burning::active_vision::Camera::Dim;

namespace scrap_burning {
  namespace active_vision {
    double computeRayQuality(const OctoRay &r, const Map *tree, const KeyMap &proxMap, double rayLength) {
      double ret = 0.0;
      double prevViewProb = 1.0;
  
      float dt = getDiscStepVal(r, tree);
      for(float t = 0.0; t <= rayLength; t += dt) {
	octomap::point3d pos = r(t);

	// Get proximity probability
	double proxProb = 0.0;
	auto iter = proxMap.find(tree->coordToKey(pos));
	if(iter != proxMap.end())	// Cell is either not a red cell or too far away
	  proxProb = iter->second;

	// Calculate occupancy probability
	double occProb = 0.5;
	octomap::ColorOcTreeNode *loc = tree->search(pos);
	if(loc != NULL)
	  occProb = loc->getOccupancy();
    
	// Calculate information gain
	double infGain = 0.0;
	if(occProb > 0 && occProb < 1)
	  double infGain = -occProb * log(occProb) - (1 - occProb) * log(1 - occProb);

	// Multiply by other factors
	ret += (infGain * prevViewProb * proxProb);

	// Set prevViewProb for next iteration
	prevViewProb *= (1 - occProb);
      }

      return ret;
    }

    std::pair<std::vector<double>, std::vector<octomap::OcTreeKey>> getRayCellScores(const OctoRay &r, const Map *tree, const KeyMap &proxMap, double rayLength) {
      std::pair<std::vector<double>, std::vector<octomap::OcTreeKey>> ret;
      double prevViewProb = 1.0;
  
      float dt = getDiscStepVal(r, tree);
      for(float t = 0.0; t <= rayLength; t += dt) {
	octomap::point3d pos = r(t);

	// Get proximity probability
	auto iter = proxMap.find(tree->coordToKey(pos));
	if(iter == proxMap.end()) {	// Cell is either not a red cell or too far away
	  ret.first.push_back(0);
	  ret.second.push_back(tree->coordToKey(pos));
	  continue;
	}
	double proxProb = iter->second;

	// Calculate occupancy probability
	double occProb = 0.5;
	octomap::ColorOcTreeNode *loc = tree->search(pos);
	if(loc != NULL)
	  occProb = loc->getOccupancy();
    
	// Calculate information gain
	if(occProb == 0 || occProb == 1.0) {
	  ret.first.push_back(0);
	  ret.second.push_back(tree->coordToKey(pos));
	  continue;
	}
	double infGain = -occProb * log(occProb) - (1 - occProb) * log(1 - occProb);

	// Multiply by other factors
	ret.first.push_back(infGain * prevViewProb * proxProb);
	ret.second.push_back(tree->coordToKey(pos));

	// Set prevViewProb for next iteration
	prevViewProb *= (1 - occProb);
      }

      return ret;
    }

    double evalViewpoint(const Camera &cam, const Map *tree, KeyMap &cache, const KeyMap &proxMap, double rayLength) {
      double ret = 0.0;
      Dim w = cam.getDescription().width;
      Dim h = cam.getDescription().height;
      octomap::point3d camPt(tree->keyToCoord(cam.getDiscretePosition(tree)));
      std::vector<Point> keys;
      for(Dim x = 0; x < w; ++x) {
	for(Dim y = 0; y < h; ++y) {
	  Point pt = cam.cast(x, y)(rayLength);
	  octomap::OcTreeKey tgtKey(tree->coordToKey(pt[0], pt[1], pt[2]));
	  auto iter = cache.find(tgtKey);
	  if(iter == cache.end()) {
	    double score = computeRayQuality(OctoRay(camPt, tree->keyToCoord(tgtKey)),
					     tree, proxMap, rayLength);
	    cache[tgtKey] = score;
	    ret += score;
	  }
	  else
	    ret += iter->second;
	}
      }
      return ret;
    }
  } // active_vision
}   // scrap_burning
