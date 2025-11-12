#include <string>
#include <fstream>

#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"

// First: KNOWN NEIGHBOUR COUNT
// Second: KNOWN OCCUPIED NEIGHBOUR COUNT
using NeighbourCount = std::tuple<uint8_t, uint8_t, uint8_t>;
enum Neighbour {UNKNOWN=0, OCCUPIED=1, UNOCCUPIED=2};

NeighbourCount getNNC(const octomap::ColorOcTree *const tree, const octomap::point3d &coord) {
  const static octomap::point3d OFFSETS[] = {{1, 0, 0}, {-1, 0, 0},
					     {0, 1, 0}, {0, -1, 0},
					     {0, 0, 1}, {0, 0, -1}};
  NeighbourCount ret{0, 0, 0};
  
  for(int x = -1; x <= 1; ++x)
    for(int y = -1; y <= 1; ++y)
      for(int z = -1; z <= 1; ++z) {
  	if(x == 0 && y == 0 && z == 0)
  	  continue;
	auto node = tree->search(coord + octomap::point3d(x, y, z) * tree->getResolution(), 0);
	std::get<UNKNOWN>(ret) += (node == NULL);
	std::get<OCCUPIED>(ret) += (node != NULL && node->getOccupancy() >= 0.5);
	std::get<UNOCCUPIED>(ret) += (node != NULL && node->getOccupancy() < 0.5);
      }

  return ret;
}

constexpr static uint8_t LINE_R = 255;
constexpr static uint8_t LINE_G = 0;
constexpr static uint8_t LINE_B = 0;

int main() {
  const std::string PATH("/home/fadi/scrap_ws/octreeFiltered.ot");

  octomap::ColorOcTree *tree = dynamic_cast<octomap::ColorOcTree*>(octomap::ColorOcTree::read(PATH));
  // Create another tree with the same resolution, and add nodes with a certain max known neighbours
  constexpr uint8_t UK_THRESH(10);
  constexpr uint8_t KO_THRESH(6);
  constexpr uint8_t KU_THRESH(40);
  // This function checks the pair agains the thresholds
  auto cmp = [&UK_THRESH, &KO_THRESH, &KU_THRESH](const NeighbourCount &val) {
	       return
		 std::get<UNKNOWN>(val) >= UK_THRESH &&
		 std::get<OCCUPIED>(val) <= KO_THRESH &&
		 std::get<UNOCCUPIED>(val) <= KU_THRESH;
	     };
  octomap::ColorOcTree frontier(tree->getResolution());
  for(auto it = tree->begin_leafs(0), end=tree->end_leafs(); it != end; ++it) {
    octomap::ColorOcTreeNode::Color c = it->getColor();
    if(c.r != LINE_R || c.g != LINE_G || c.b != LINE_B) // Check if it is a line voxel
      continue;
    if(cmp(getNNC(tree, it.getCoordinate())))
      frontier.insertRay({0, 0, 0}, it.getCoordinate(), LINE_R, LINE_G, LINE_B, -1.0, false);
    else
      frontier.insertRay({0, 0, 0}, it.getCoordinate(), LINE_G, LINE_R, LINE_B, -1.0, false);
  }

  // Write the frontier
  frontier.write("/home/fadi/scrap_ws/octree4.ot");

  return 0;
}
