#include <fstream>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "scrap_burning/FilterPath.h"
#include "scrap_burning/CurveFitting.h"
#include "ros_pcl_manip/Downsample.h"
#include "scrap_burning/pcl.hpp"
#include "scrap_burning/config/config.hpp"

// Benchmark includes
#include "traj_reader.hpp"
#include "timing_reader.hpp"
#include "cloud_processing.hpp"

using scrap_burning::pcl::PointType;
using scrap_burning::pcl::CloudType;
using scrap_burning::pcl::CloudPtr;

static constexpr double LEAF_SIZE = 0.01;

// Helper function
std::vector<double> computeDistances(const std::vector<Eigen::Isometry3d> &pts) {
  std::vector<double> ret;

  if(pts.size() == 0) return ret;

  ret.reserve(pts.size());

  const Eigen::Isometry3d *prevPose = &pts[0];
  for(const auto &pt : pts) {
    // Compute delta
    Eigen::Vector3d deltaPos = pt.translation() - prevPose->translation();
    ret.push_back(deltaPos.norm());

    // Reassign prevPose to the current one
    prevPose = &pt;
  }

  return ret;
}

double calcTotalTime(std::vector<TimingEntry>::const_iterator cbegin,
		     std::vector<TimingEntry>::const_iterator cend) {
  return std::accumulate(cbegin, cend, 0.0,
			 [](double total, auto &entry) {
			   return total + entry.getDuration();
			 });
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "coverage_calculator_node");
  ros::NodeHandle nh;

  if(argc != 6) {
    std::cerr << "Usage: " << argv[0] << " ground_truth_cloud.pcd measured_cloud.pcd trajectory.txt timing.txt output_directory\n";
    return 1;
  }

  ROS_INFO("Loading input files====================================");

  ROS_INFO("Loading .pcd files");
  CloudPtr groundTruthCloud(new CloudType());
  if(pcl::io::loadPCDFile(argv[1], *groundTruthCloud) == -1) {
    std::cerr << "Failed to load measured pcd file: " << argv[1] << '\n';
    return 2;
  }
  CloudPtr measuredCloud(new CloudType());
  if(pcl::io::loadPCDFile(argv[2], *measuredCloud) == -1) {
    std::cerr << "Failed to load ground truth pcd file: " << argv[2] << '\n';
    return 2;
  }

  ROS_INFO("Loading trajectory file");
  auto trajectory = readTrajFile(argv[3]);
  ROS_INFO_STREAM("Found " << trajectory.second.size() << " waypoints");

  ROS_INFO("Loading timing file");
  auto timing = readTimingFile(argv[4]);

  // Output directory
  std::string outputDir = argv[5];
  if(outputDir.back() != '/')
    outputDir.push_back('/');

  ROS_INFO("Loaded all files");

  ROS_INFO_STREAM("Outputing files to " << outputDir << "metrics.txt");
  std::ofstream ofs(outputDir + "metrics.txt");

  // Load config
  ROS_INFO("Loading config");
  scrap_burning::ScrapBurningConfig sbc(nh);
  sbc.loadParams();
  ROS_INFO("Loaded config");
  
  ROS_INFO("Computing Coverage=====================================");

  ROS_INFO("Cleaning clouds");
  // First, do some cleanup
  // Filter bounds to remove the walls/ceiling/floor
  const Eigen::Vector3f minBounds(0.0, -2.0, 0.5);
  // const Eigen::Vector3f maxBounds(2.0, 2.0, 2.5);
  const Eigen::Vector3f maxBounds(0.85, 2.0, 2.5);
  const int minPts = 3000;
  measuredCloud = filterCloudOutsideBounds(measuredCloud, minBounds, maxBounds);
  measuredCloud = removeOutliers(measuredCloud, minPts);
  groundTruthCloud = filterCloudOutsideBounds(groundTruthCloud, minBounds, maxBounds);
  groundTruthCloud = removeOutliers(groundTruthCloud, minPts);

  ROS_INFO_STREAM("Saving cleaned measured cloud to " << outputDir << "cleanedMeasured.pcd");
  pcl::io::savePCDFileASCII(outputDir + "cleanedMeasured.pcd", *measuredCloud);
  ROS_INFO_STREAM("Saving cleaned ground truth cloud to " << outputDir << "cleanedGroundTruth.pcd");
  pcl::io::savePCDFileASCII(outputDir + "cleanedGroundTruth.pcd", *groundTruthCloud);

  // First, filter both clouds
  CloudPtr fMeasuredCloud = filterCloud(nh, sbc, measuredCloud);
  CloudPtr fGroundTruthCloud = filterCloud(nh, sbc, groundTruthCloud);
  // Check if the filter worked
  if(fMeasuredCloud->size() == 0 || fGroundTruthCloud->size() == 0) {
    ROS_ERROR("Could not filter the line on either the measured or ground truth cloud");
    return 3;
  }

  ROS_INFO_STREAM("Saving cleaned measured filtered line cloud to " << outputDir << "cleanedFiltered.pcd");
  pcl::io::savePCDFileASCII(outputDir + "cleanedMeasuredFiltered.pcd", *fMeasuredCloud);

  // Next, downsample the line
  CloudPtr dfMeasuredCloud = downsampleCloud(nh, LEAF_SIZE, fMeasuredCloud);
  CloudPtr dfGroundTruthCloud = downsampleCloud(nh, LEAF_SIZE, fGroundTruthCloud);
  // Finally, downsample the unfiltered cloud for display
  CloudPtr dMeasuredCloud = downsampleCloud(nh, LEAF_SIZE, measuredCloud);
  CloudPtr dGroundTruthCloud = downsampleCloud(nh, LEAF_SIZE, groundTruthCloud);

  // Get the indices of the ground truth line that were found
  ROS_INFO("Computing coverage");
  pcl::PointIndices::Ptr groundTruthMatch = computeMatches(dfMeasuredCloud, dfGroundTruthCloud, LEAF_SIZE * 2);
  ROS_INFO_STREAM("Coverage: " << groundTruthMatch->indices.size() << " / " << dfGroundTruthCloud->size());
  ofs << "Coverage: " << groundTruthMatch->indices.size() << " / " << dfGroundTruthCloud->size() << '\n';

  // Extract the found points
  CloudPtr matchedPts = extractIndices(dfGroundTruthCloud, groundTruthMatch);

  // Color the original ground truth pointcloud
  ROS_INFO("Coloring point cloud");
  // Color the line red
  colorCloud(dGroundTruthCloud, 255, 0, 0, computeMatches(dfGroundTruthCloud, dGroundTruthCloud, 0.0001));
  // Color all found points green
  colorCloud(dGroundTruthCloud, 0, 255, 0, computeMatches(matchedPts, dGroundTruthCloud, LEAF_SIZE / 2));

  ROS_INFO_STREAM("Saving colored coverage cloud to " << outputDir << "coverage.pcd");
  pcl::io::savePCDFileASCII(outputDir + "coverage.pcd", *dGroundTruthCloud);

  ROS_INFO("Computing Accuracy=====================================");

  std::vector<float> distances = getNearestCorrespondences(fMeasuredCloud, fGroundTruthCloud);
  // Compute average
  double distTotal = std::accumulate(distances.begin(), distances.end(), 0.0);
  ROS_INFO_STREAM("Total error: " << distTotal << 'm');
  ROS_INFO_STREAM("Average error: " << distTotal / measuredCloud->size() << 'm');
  ofs << "Total error: " << distTotal << "m\n";
  ofs << "Average error: " << distTotal / measuredCloud->size() << "m\n";

  ROS_INFO("Computing Trajectory Information=======================");

  std::vector<double> trajDeltas = computeDistances(trajectory.first);
  std::vector<double> waypointDeltas = computeDistances(trajectory.second);
  ROS_INFO_STREAM("Total distance traveled: " << std::accumulate(trajDeltas.begin(), trajDeltas.end(), 0.0));
  ROS_INFO_STREAM("Total distance traveled between waypoints: " << std::accumulate(waypointDeltas.begin(), waypointDeltas.end(), 0.0));
  ofs << "Total distance traveled: " << std::accumulate(trajDeltas.begin(), trajDeltas.end(), 0.0) << '\n';
  ofs << "Total distance traveled between waypoints: " << std::accumulate(waypointDeltas.begin(), waypointDeltas.end(), 0.0) << '\n';

  ROS_INFO("Computing Timing Information===========================");

  ROS_INFO("Computing total time");
  double timingTotal = calcTotalTime(timing.cbegin(), timing.cend());
  ROS_INFO_STREAM("Total time: " << timingTotal);
  ROS_INFO_STREAM("Average time per iteration: " << timingTotal / timing.size());
  ofs << "Total time: " << timingTotal << '\n';
  ofs << "Average time per iteration: " << timingTotal / timing.size() << '\n';

  ROS_INFO("Computing total time without robot movement");
  const double trajRate = 10;
  const double threshold = 0.0005;
  std::vector<TimingEntry> adjustedDurations;
  auto offset = trajDeltas.cbegin();
  for(const auto &timingEntry : timing) {
    // Get the indices which correspond to this timingEntry's interval
    auto intervalEnd = offset + static_cast<int>(timingEntry.getDuration() * trajRate);
    // Clamp it
    if((intervalEnd - trajDeltas.cbegin()) > (trajDeltas.cend() - trajDeltas.cbegin()))
      intervalEnd = trajDeltas.cend();

    // Interval is offset -> intervalEnd
    double adjustedTime = std::accumulate(offset, intervalEnd, timingEntry.getDuration(),
					  [trajRate, threshold](double totTime, double dist) {
					    if(dist > threshold)
					      totTime -= (1 / trajRate);
					    return totTime;
					  });
    adjustedDurations.push_back(TimingEntry(timingEntry.getFrom(), adjustedTime));

    offset = intervalEnd;
  }

  double noMovementTime = calcTotalTime(adjustedDurations.cbegin(), adjustedDurations.cend());
  ROS_INFO_STREAM("Total time without movement: " << noMovementTime);
  ROS_INFO_STREAM("Average time per iteration without movement: " << noMovementTime / adjustedDurations.size());
  ofs << "Total time without movement: " << noMovementTime << '\n';
  ofs << "Average time per iteration without movement: " << noMovementTime / adjustedDurations.size() << '\n';

  return 0;
}
