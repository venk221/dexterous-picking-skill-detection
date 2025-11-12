#include <cmath>
#include <string>
#include <random>
#include <fstream>
#include <numeric>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "conveyor_sorting/utils.hpp"
#include "conveyor_sorting/config.hpp"
#include "conveyor_sorting_msgs/Sweep.h"
#include "conveyor_sorting_msgs/SweepingAction.h"

#include "conveyor_sorting/functors.hpp"

#include <qnd/vis.hpp>
#include <qnd/cam.hpp>
#include <qnd/geom.hpp>
#include <qnd/comm.hpp>
#include <qnd/moveit.hpp>

// ENABLE DEBUG PRINTING
static constexpr bool debugPrinting{true};
// Image type for the depth data
static constexpr char depthType[]{"32FC1"};
// Image type for the color data
static constexpr char colorType[]{"bgr8"};

// Used if we need to generate a new seed
static std::random_device rd{};

std::vector<double> generateDoubles(double start, double end, int cnt) {
  std::vector<double> ret;
  ret.reserve(cnt);

  for(double val{start}; val <= end; val += ((end - start) / cnt)) {
    ret.push_back(val);
  }

  return ret;
}

template <typename Application, typename Mover, typename SweepingExecutor, typename Scorer>
bool runApp(Application& app, std::shared_ptr<qnd::RosCam> topCam, std::shared_ptr<qnd::RosCam> colCam,
	    Mover& mover, SweepingExecutor& executor, Scorer scorer,
	    bool execute, const std::vector<double>& initialJoints, std::size_t iter) {
  cv::Mat topImg{};
  cv::Mat rgbImg{};

  // Output data to properly visualize things later on
  std::ofstream ofs{"/tmp/output.csv"};

  // Write topographical map heights
  ROS_INFO("Getting topographical map heights");
  std::vector<double> topMapHeights(app.getSweeper().getTopMapHeights());
  for(std::size_t i = 0; i < topMapHeights.size(); ++i) {
    ofs << topMapHeights[i];
    if(i < topMapHeights.size() - 1)
      ofs << ',';
  }
  ofs << '\n';

  ROS_INFO("Wrote primary data to output file, beginning iterations");
  for(std::size_t curIter = 0; curIter < iter && ros::ok(); ++curIter) {
    ROS_INFO_STREAM("Currently at iteration " << curIter + 1 << " / " << iter);

    if(!mover(initialJoints, execute)) {
      ROS_FATAL("Failed to move to initial joint configuration");
      return false;
    }

    topImg = topCam->getLatestImgCV();
    rgbImg = colCam->getLatestImgCV();

    std::pair<cv::Mat, double> img_score_pair{scorer()};

    ofs << img_score_pair.second << ',';

    ROS_INFO("Generating new sweeping action");
    conveyor_sorting_msgs::SweepingAction action(app(rgbImg, topImg));

    // Output sweeping action details
    ofs << qnd::toString(app.getProjector().deproject(action.start), ",") <<
      ',' << qnd::toString(app.getProjector().deproject(action.end), ",") << ',';
    // Output images
    cv::imwrite(std::string("/tmp/") + std::to_string(curIter) + "rgb.png", rgbImg * 255.0);
    cv::imwrite(std::string("/tmp/") + std::to_string(curIter) + "top.png", topImg);
    cv::imwrite(std::string("/tmp/") + std::to_string(curIter) + "pre_score.png", img_score_pair.first);

    // If the action is invalid/empty, there was an error, and we should skip this run
    if(action.start == action.end) {
      ROS_WARN_STREAM("Invalid action returned from app, skipping run " << curIter + 1);
    }
    else {
      ROS_INFO("Executing sweeping action");
      if(!executor(action)) {
	ROS_FATAL("Failed to execute sweeping action");
	return false;
      }
    }

    img_score_pair = scorer();
    ofs << img_score_pair.second << '\n';

    cv::imwrite(std::string("/tmp/") + std::to_string(curIter) + "post_score.png", img_score_pair.first);

    ros::spinOnce();
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ot_sweeper_node");
  ros::NodeHandle nh{};

  // Enable debug printing
  if(debugPrinting) {
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  ConfigLoader cfgLoader{nh};
  if(!cfgLoader.loadParams()) {
    std::cerr << "Failed to load all params\n";
    return 1;
  }

  // The configs we care about
  const TopographicalMapConfig& topCfg{cfgLoader.getConfig().topographical_map};
  const SweeperConfig& sweepCfg{cfgLoader.getConfig().sweeper};

  // A visualizer
  qnd::Visualizer vis{nh};

  // List of heights in the topographical map
  std::vector<double> topMapHeights = generateDoubles(topCfg.peak_distance / topCfg.division_count,
						      topCfg.peak_distance, topCfg.division_count - 1);

  // The two cameras we will be using
  const std::string& camIntrinsics{sweepCfg.camera_intrinsics_topic};
  std::shared_ptr<qnd::RosCam> topCam(std::make_shared<qnd::RosCam>(nh, topCfg.topographical_map_topic, camIntrinsics, depthType));
  std::shared_ptr<qnd::RosCam> rgbCam(std::make_shared<qnd::RosCam>(nh, topCfg.input_color_topic, camIntrinsics, topCfg.input_color_encoding));

  // Get an initial couple of images
  topCam->waitForImage();
  rgbCam->waitForImage();

  cv::Mat topImg{topCam->getLatestImgCV()};
  cv::Mat rgbImg{rgbCam->getLatestImgCV()};

  // Some motion planning data
  const std::vector<double>& initialJoints(cfgLoader.getConfig().motion_planner.initial_joints);
  bool execute(cfgLoader.getConfig().motion_planner.execute);

  // Build a projector that converts from the image into the world
  // conveyor_sorting::TopographicalProjector projector(topCam, topCfg.camera_table_offset, "world");
  conveyor_sorting::OffsetProjector projector(topCfg.camera_table_offset);
  // Build a scorer that can give us the score at the end of each sweep
  conveyor_sorting::Scorer scorer(nh, topCfg.score_topic, topCfg.score_img_topic);
  // Build a sweeping action executor to actually sweep
  conveyor_sorting::SweepExecutor actionExecutor(nh, sweepCfg.sweep_topic);

  ROS_INFO("Initializing functors");
  // These are the functors used by the application
  conveyor_sorting::ImgProcessor imgProcessor{conveyor_sorting::AreaWeightedRandomSampler{0},
  					      conveyor_sorting::ROIHammersleySampler(topMapHeights[0],
                                                         topCfg.roi.top_left.x, topCfg.roi.top_left.y,
                                                         topCfg.roi.bottom_right.x - topCfg.roi.top_left.x,
                                                         topCfg.roi.bottom_right.y - topCfg.roi.top_left.y),
					      topMapHeights, static_cast<std::size_t>(sweepCfg.ot_sweeper.samples)
  };
  conveyor_sorting::DistanceCost costCalculator{};
  conveyor_sorting::NodeOTCalculator otCalculator{nh};
  // Possible sweepers
  conveyor_sorting::DotProductSweeper dpSweeper{static_cast<std::size_t>(sweepCfg.discrete_angles),
						topMapHeights, {topImg.rows, topImg.cols}};
  // If we pass in a fixed seed, use that, otherwise generate a random one
  int seed = sweepCfg.seed;
  if(seed == -1)
    seed = rd();
  conveyor_sorting::RandomSweeper randSweeper{static_cast<std::size_t>(sweepCfg.discrete_angles),
					      topMapHeights, {topImg.rows, topImg.cols}, seed};
  conveyor_sorting::CSweeper cSweeper{static_cast<std::size_t>(sweepCfg.discrete_angles),
    topMapHeights, qnd::createPt(sweepCfg.motion.width, 0.0, sweepCfg.motion.height)};
  ROS_INFO("Created all functors, beginning main loop");

  // Build a motion planner that can move the robot
  if(sweepCfg.move) {
    conveyor_sorting::Mover mover(nh);

    if(!mover.addCollision(qnd::createCollisionBox("ot_sweeper_table",
						   cfgLoader.getConfig().table_desc.center,
						   cfgLoader.getConfig().table_desc.size))) {
      ROS_FATAL("Failed to add table collision object");
      return 3;
    }
  }

  if(sweepCfg.use_ot) {
    conveyor_sorting::Application otApp{imgProcessor, costCalculator, cSweeper, otCalculator,
					projector, vis, true};
    if(sweepCfg.move) {
      conveyor_sorting::Mover mover(nh);

      if(!mover.addCollision(qnd::createCollisionBox("ot_sweeper_table",
						     cfgLoader.getConfig().table_desc.center,
						     cfgLoader.getConfig().table_desc.size))) {
	ROS_FATAL("Failed to add table collision object");
	return 3;
      }
      return !runApp(otApp, topCam, rgbCam, mover, actionExecutor, scorer, execute, initialJoints, sweepCfg.max_iter);
    }
    else {
      conveyor_sorting::EmptyMover mover{};
      return !runApp(otApp, topCam, rgbCam, mover, actionExecutor, scorer, execute, initialJoints, sweepCfg.max_iter);
    }
  }
  else {
    conveyor_sorting::Application otApp{imgProcessor, costCalculator, randSweeper, otCalculator,
					projector, vis, true};
    if(sweepCfg.move) {
      conveyor_sorting::Mover mover(nh);

      if(!mover.addCollision(qnd::createCollisionBox("ot_sweeper_table",
						     cfgLoader.getConfig().table_desc.center,
						     cfgLoader.getConfig().table_desc.size))) {
	ROS_FATAL("Failed to add table collision object");
	return 3;
      }
      return !runApp(otApp, topCam, rgbCam, mover, actionExecutor, scorer, execute, initialJoints, sweepCfg.max_iter);
    }
    else {
      conveyor_sorting::EmptyMover mover{};
      return !runApp(otApp, topCam, rgbCam, mover, actionExecutor, scorer, execute, initialJoints, sweepCfg.max_iter);
    }
  }

}
