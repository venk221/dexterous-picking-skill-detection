#include <cmath>

#include "scrap_burning/active_vision/grid.hpp"

typedef scrap_burning::active_vision::Map Map;
typedef scrap_burning::active_vision::NeighbourCount NeighbourCount;
typedef scrap_burning::active_vision::Neighbour Neighbour;
typedef scrap_burning::active_vision::RayCells RayCells;
typedef scrap_burning::active_vision::Ray Ray;

std::vector<octomap::OcTreeKey> scrap_burning::active_vision::getNeighbours(const octomap::OcTreeKey &key, Connectivity con) {
  static constexpr std::array<std::array<int, 3>, 26> DELTAS
    {
     {
      {-1,  0,  0}, { 1,  0, 0}, { 0, -1,  0}, { 0,  1, 0}, {0,  0, -1}, {0,  0, 1},
      {-1, -1,  0}, {-1,  1, 0}, {-1,  0, -1}, {-1,  0, 1}, {1, -1,  0}, {1,  1, 0},
      { 1,  0, -1}, { 1,  0, 1}, { 0, -1, -1}, { 0, -1, 1}, {0,  1, -1}, {0,  1, 1},
      {-1, -1, -1}, {-1, -1, 1}, {-1,  1, -1}, {-1,  1, 1}, {1, -1, -1}, {1, -1, 1},
      { 1,  1, -1}, { 1,  1, 1}
     }
    };

  int neighbourCount = static_cast<int>(con);
  std::vector<octomap::OcTreeKey> ret;
  ret.reserve(neighbourCount);

  for(int i = 0; i < neighbourCount; ++i)
    ret.push_back(octomap::OcTreeKey(key[0] + DELTAS[i][0],
				     key[1] + DELTAS[i][1],
				     key[2] + DELTAS[i][2]));

  return ret;
}

NeighbourCount scrap_burning::active_vision::getNNC(const Map *tree, const octomap::OcTreeKey &coord, Connectivity con) {
  NeighbourCount ret{0, 0, 0};
  for(const octomap::OcTreeKey &cell : getNeighbours(coord, con)) {
    auto node = tree->search(cell, 0);
    std::get<UNKNOWN>(ret) += (node == NULL);
    std::get<OCCUPIED>(ret) += (node != NULL && node->getOccupancy() >= 0.5);
    std::get<UNOCCUPIED>(ret) += (node != NULL && node->getOccupancy() < 0.5);
  }

  return ret;
}

RayCells scrap_burning::active_vision::discretize(const Ray &r, const Map *tree, double l) {
  RayCells ret;

  auto cellPos = posDiscretize(r, tree, l);
  ret.reserve(cellPos.size());

  for(const auto &pos : cellPos)
    ret.push_back({pos, tree->search(pos[0], pos[1], pos[2])});

  return ret;
}

float scrap_burning::active_vision::getDiscStepVal(const OctoRay &r, const Map *tree) {
  float maxDiff = std::max(std::abs(r.getVector().x()),
			   std::max(std::abs(r.getVector().y()),
				    std::abs(r.getVector().z())));
  return (r.getVector() * (tree->getResolution() / maxDiff)).norm();
}

float scrap_burning::active_vision::getDiscStepVal(const Ray &r, const Map *tree) {
  float maxDiff = 0, minDiff = 10;
  for(int i = 0; i < 3; ++i) {
    maxDiff = std::max(r.getVector()[i], maxDiff);
    minDiff = std::min(r.getVector()[i], minDiff);
  }
  float diff = (maxDiff > std::abs(minDiff) ? maxDiff : minDiff);
  return (r.getVector() * (tree->getResolution() / diff)).norm();
}

std::vector<scrap_burning::active_vision::Point> scrap_burning::active_vision::posDiscretize(const Ray &r,
											     const Map *tree,
											     double l) {
  double res = tree->getResolution();
  std::vector<Point> ret;
  // Check the longest component
  // Once we find that, that component will need to move forward by 'res'
  // We can then calculate the magnitude of the full distance along the ray with that
  float dt = getDiscStepVal(r, tree);
  std::cout << "Discretizing with a delta of " << dt << " across " << l << " total length";
  ret.reserve(l / dt + 2);

  for(float t = 0.0; t <= l; t += dt) {
    auto cur(r(t));
    ret.push_back(cur);
  }

  return ret;
}

void scrap_burning::active_vision::draw(const RayCells &cells, Map *tree, const octomap::ColorOcTreeNode::Color &c) {
  for(const auto &posNode : cells) {
    const auto &pos = posNode.first;
    tree->setNodeValue(pos[0], pos[1], pos[2], 1.0);
    tree->updateNode(pos[0], pos[1], pos[2], true)->getColor() = c;
  }
}
void scrap_burning::active_vision::draw(const std::vector<octomap::OcTreeKey> &keys, Map *tree, const octomap::ColorOcTreeNode::Color &c) {
  for(const auto &key : keys) {
    auto node = tree->updateNode(key, true);
    node->setColor(c);
  }
}

double scrap_burning::active_vision::getDist(const PositionedCell &from, const PositionedCell &to) {
  return (to.first - from.first).norm();
}

double scrap_burning::active_vision::getShortestDist(const PositionedCell &cell, const RayCells &cells) {
  double minDist = 100000.0;
  for(const auto &destCell : cells) {
    double dist(getDist(cell, destCell));
    if(dist < minDist) minDist = dist;
  }

  return minDist;
}

std::vector<octomap::OcTreeKey> scrap_burning::active_vision::sphereCast(const octomap::OcTreeKey &from, double rad,
									 const Map *tree) {
  typedef std::tuple<int, int, int> KeyDelta;

  std::vector<octomap::OcTreeKey> ret;
  rad /= (tree->getResolution());
  auto fillSigns = [](KeyDelta loc, std::vector<KeyDelta> &vec) {
		     int x = std::get<0>(loc), y = std::get<1>(loc), z = std::get<2>(loc);
		     vec.push_back({x, y, z});
		     while(true) {
		       z = -z;
		       if(z >= 0) {
			 y = -y;
			 if(y >= 0) {
			   x = -x;
			   if(x >= 0)
			     break;
			 }
		       }
		       vec.push_back({x, y, z});
		     }
		   };
  auto fillAll = [&fillSigns](const KeyDelta &loc, std::vector<KeyDelta> &vec) {
		   fillSigns(loc, vec);
		   if(std::get<2>(loc) > std::get<1>(loc))
		     fillSigns({std::get<0>(loc), std::get<2>(loc), std::get<1>(loc)}, vec);
		   if(std::get<2>(loc) > std::get<0>(loc) && std::get<2>(loc) > std::get<1>(loc))
		     fillSigns({std::get<2>(loc), std::get<1>(loc), std::get<0>(loc)}, vec);
		 };
  // Construct sphere on the shape
  int maxRad = floor(rad * rad);
  int zx = floor(rad);
  for(int x = 0; ; ++x) {
    while(x * x + zx * zx > maxRad && zx >= x)
      --zx;
    if(zx < x)
      break;

    int z = zx;
    for(int y = 0; ; ++y) {
      while(x * x + y * y + z * z > maxRad && z >= x && z >= y)
	--z;
      if(z < x || z < y)
	break;
      std::vector<KeyDelta> deltas;
      fillAll({x, y, z}, deltas);
      for(const KeyDelta &delta : deltas) {
	ret.push_back(octomap::OcTreeKey(from[0] + std::get<0>(delta),
					 from[1] + std::get<1>(delta),
					 from[2] + std::get<2>(delta)));
      }
    }
  }

  return ret;
}

std::vector<octomap::OcTreeKey> scrap_burning::active_vision::genGrid(std::vector<octomap::OcTreeKey> start, int cnt) {
  KeySet existingCells;
  existingCells.insert(start.begin(), start.end());
  std::vector<octomap::OcTreeKey> next;
  for(int i = 0; i < cnt; ++i) {
    // Expand each cell in start towards a new direction
    for(const octomap::OcTreeKey &key : start) {
      for(int dX = -1; dX <= 1; ++dX)
	for(int dY = -1; dY <= 1; ++dY)
	  for(int dZ = -1; dZ <= 1; ++dZ) {
	    octomap::OcTreeKey dKey(key[0] + dX, key[1] + dY, key[2] + dZ);
	      if(existingCells.find(dKey) == existingCells.end()) { // New key, not encountered before
		existingCells.insert(dKey);
		next.push_back(dKey);
	      }
	  }
    }

    // Swap and reset
    std::swap(next, start);
    // Clear it for insertion on the next loop if we will iterate again
    if(i < cnt - 1)
      next.clear();
  }

  return next;
}
