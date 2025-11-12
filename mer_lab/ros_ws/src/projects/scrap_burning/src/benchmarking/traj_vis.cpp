#include <cstdio>
#include <string>
#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>

#include "scrap_burning/pcl.hpp"
#include "traj_reader.hpp"

static constexpr double LINE_R    = 1.0;
static constexpr double LINE_G    = 0.0;
static constexpr double LINE_B    = 1.0;

static constexpr double WAYPT_R   = 0.0;
static constexpr double WAYPT_G   = 1.0;
static constexpr double WAYPT_B   = 0.0;

static constexpr double ARROW_LEN = 0.25;

// Helper function to convert from eigen points to pcl points
pcl::PointXYZ convertEigen(const Eigen::Vector3d &vec) {
  return pcl::PointXYZ(vec[0], vec[1], vec[2]);
}

int main(int argc, char **argv) {
  if(argc < 3) {
    std::cerr << "Usage: " << argv[0] << " trajectory cloud [--waypoints_only] [--draw_circles]\n";
    return 1;
  }

  // Get trajectory
  auto traj = readTrajFile(argv[1]);
  std::vector<Eigen::Isometry3d> &pts = traj.first;
  std::vector<Eigen::Isometry3d> &waypts = traj.second;

  // Read cloud
  scrap_burning::pcl::CloudPtr cloud(new scrap_burning::pcl::CloudType());
  if(pcl::io::loadPCDFile(argv[2], *cloud) == -1) {
    std::cerr << "Failed to read point cloud from file " << argv[2] << '\n';
    return 2;
  }

  // Check for any optional flags
  // Flag that decides whether we draw the full trajectory or only the waypoints
  bool drawFull = true;
  // Flag that decides whether we draw arrows or circles for the waypoints
  bool drawCircles = false;
  for(int argNum = 3; argNum < argc; ++argNum) {
    if(strcmp(argv[argNum], "--waypoints_only") == 0)
      drawFull = false;
    if(strcmp(argv[argNum], "--draw_spheres") == 0)
      drawCircles = true;
  }

  pcl::visualization::PCLVisualizer vis;
  vis.addPointCloud(cloud, "Collected");

  // Decide what to draw based on drawFull
  std::vector<Eigen::Isometry3d> *toDraw = nullptr;
  if(drawFull) toDraw = &pts;
  else toDraw = &waypts;

  for(std::size_t i = 0; i < toDraw->size() - 1; ++i) {
    std::ostringstream oss;
    oss << i << " -> " << i + 1;
    vis.addLine(convertEigen((*toDraw)[i].translation()),
		convertEigen((*toDraw)[i + 1].translation()),
		LINE_R, LINE_G, LINE_B, oss.str());
  }
  std::cout << "Finished adding points" << std::endl;

  // Add waypoints
  for(std::size_t i = 0; i < waypts.size(); ++i) {
    std::stringstream oss;
    oss << i << "_waypoints";
    if(drawCircles)
      vis.addSphere(convertEigen(waypts[i].translation()), 0.02,
		    WAYPT_R, WAYPT_G, WAYPT_B, oss.str());
    else
      vis.addArrow(convertEigen(waypts[i].linear().col(2) * ARROW_LEN + waypts[i].translation()),
		   convertEigen(waypts[i].translation()),
		   WAYPT_R, WAYPT_G, WAYPT_B, false, oss.str());
  }
  vis.spin();

  return 0;
}
