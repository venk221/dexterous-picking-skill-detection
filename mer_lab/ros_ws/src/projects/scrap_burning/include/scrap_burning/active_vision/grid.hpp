#ifndef ACTIVE_VISION_GRID_HPP
#define ACTIVE_VISION_GRID_HPP

#include "core.hpp"

#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace scrap_burning {
  namespace active_vision {

    typedef std::unordered_map<octomap::OcTreeKey, double, octomap::OcTreeKey::KeyHash> KeyMap;
    typedef std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> KeySet;

    // Datatypes that store the nearest-neighbour information of cells
    typedef std::tuple<uint8_t, uint8_t, uint8_t> NeighbourCount;
    enum Neighbour {UNKNOWN=0, OCCUPIED=1, UNOCCUPIED=2};

    // Returns a vector containing the keys of the cell's neighbours
    // The connectivity enum is used to decide what kind of neighbourhood is required
    enum class Connectivity {Face = 6, Edge = 18, Corner = 26};
    std::vector<octomap::OcTreeKey> getNeighbours(const octomap::OcTreeKey &key, Connectivity con);

    // Returns the nearest-neighbour stats for a cell
    NeighbourCount getNNC(const Map *tree, const octomap::OcTreeKey &coord, Connectivity con);

    // Discretizes a ray over a grid
    RayCells discretize(const Ray &r, const Map *tree, double l=1.0);

    float getDiscStepVal(const OctoRay &r, const Map *tree);
    float getDiscStepVal(const Ray &r, const Map *tree);

    // Discretizes a ray over a grid but only returns the positions, not the Nodes
    std::vector<Point> posDiscretize(const Ray &r, const Map *tree, double l=1.0);

    // Draws a set of RayCells on a grid
    void draw(const RayCells &cells, Map *tree, const octomap::ColorOcTreeNode::Color &c=octomap::ColorOcTreeNode::Color(0, 255, 0));
    // Draws a set of Keys on a grid
    void draw(const std::vector<octomap::OcTreeKey> &keys, Map *tree, const octomap::ColorOcTreeNode::Color &c=octomap::ColorOcTreeNode::Color(0, 255, 0));

    // Returns the distance from one cell to another
    double getDist(const PositionedCell &from, const PositionedCell &to);

    // Returns the shortest distance from one Cell to a list of Cells
    double getShortestDist(const PositionedCell &cell, const RayCells &cells);

    // Returns a list of cells describing a sphere around a specific voxel
    std::vector<octomap::OcTreeKey> sphereCast(const octomap::OcTreeKey &from, double rad, const Map *tree);

    std::vector<octomap::OcTreeKey> genGrid(std::vector<octomap::OcTreeKey> start, int cnt);
  } // active_vision
}   // scrap-burning

#endif	// ACTIVE_VISION_GRID_HPP
