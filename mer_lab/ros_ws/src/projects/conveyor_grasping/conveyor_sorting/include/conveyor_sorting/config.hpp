#ifndef CONVEYOR_SORTING_CONFIG_HPP
#define CONVEYOR_SORTING_CONFIG_HPP

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// Helper function to load all configs necessary
// Structs which store the config of the project
struct Color {
  int h;
  int s;
  int v;
};
struct DesiredColorRangeConfig {
  Color min;
  Color max;
};
struct Point2D {
  int x;
  int y;
};
struct ROI {
  Point2D top_left;
  Point2D bottom_right;
};
struct TopographicalMapConfig {
  std::string raw_depth_topic;
  std::string input_depth_topic;
  std::string input_color_topic;
  std::string input_color_encoding;
  std::string topographical_map_topic;
  double camera_table_offset;
  int division_count;
  double peak_distance;
  ROI roi;
  std::string score_topic;
  std::string score_img_topic;
  DesiredColorRangeConfig desired_color_range;
};
struct TableDescConfig {
  geometry_msgs::Point center;
  geometry_msgs::Point size;
};
struct Range {
  int min;
  int max;
};
struct PileSpawnerConfig {
  double pile_width;
  double pile_length;
  double pile_height_diff;
  Range pile_items;
};
struct OTSweeperConfig {
  int samples;
  double spread_area;
};
struct SweepMotion {
  double width;
  double height;
};
struct SweeperConfig {
  SweepMotion motion;
  int max_iter;
  bool use_ot;
  bool move;
  int seed;
  int discrete_angles;
  std::string sweep_topic;
  std::string camera_intrinsics_topic;
  std::string camera_frame;
  std::string world_frame;
  OTSweeperConfig ot_sweeper;
};
struct MotionConfig {
  std::vector<double> initial_joints;
  bool execute;
};
struct ConveyorSortingConfig {
  TopographicalMapConfig topographical_map;
  TableDescConfig table_desc;
  PileSpawnerConfig pile_spawner;
  SweeperConfig sweeper;
  MotionConfig motion_planner;
};
class ConfigLoader {
public:
  ConfigLoader(ros::NodeHandle& nh)
    : _nh(nh), _config{} {}

  bool loadParams() {
    return
      _loadTopographicalMapConfig() &&
      _loadTableDescConfig() &&
      _loadPileSpawnerConfig() &&
      _loadSweeperConfig() &&
      _loadMotionConfig();
  }

  // Return the config as a const ref to:
  //  1. Avoid copying
  //  2. Prevent caller from modifying it
  const ConveyorSortingConfig& getConfig() const {
    return _config;
  }
  ros::NodeHandle& getNodeHandle() {
    return _nh;
  }
  const ros::NodeHandle& getNodeHandle() const {
    return _nh;
  }
private:
  ros::NodeHandle& _nh;
  ConveyorSortingConfig _config;

  // Helper functions
  bool _loadColorConfig(const std::string& prefix, Color& color) {
    return
      _nh.getParam(prefix + "/h", color.h) &&
      _nh.getParam(prefix + "/s", color.s) &&
      _nh.getParam(prefix + "/s", color.v);
  }
  bool _loadTopographicalMapConfig() {
    static const std::string topicPrefix{"conveyor_sorting/topographical_map"};

    TopographicalMapConfig& topConfig{_config.topographical_map};
    return
      _nh.getParam(topicPrefix + "/raw_depth_topic", topConfig.raw_depth_topic) &&
      _nh.getParam(topicPrefix + "/input_depth_topic", topConfig.input_depth_topic) &&
      _nh.getParam(topicPrefix + "/input_color_topic", topConfig.input_color_topic) &&
      _nh.getParam(topicPrefix + "/input_color_encoding", topConfig.input_color_encoding) &&
      _nh.getParam(topicPrefix + "/topographical_map_topic", topConfig.topographical_map_topic) &&
      _nh.getParam(topicPrefix + "/camera_table_offset", topConfig.camera_table_offset) &&
      _nh.getParam(topicPrefix + "/division_count", topConfig.division_count) &&
      _nh.getParam(topicPrefix + "/peak_distance", topConfig.peak_distance) &&
      _nh.getParam(topicPrefix + "/roi/top_left/x", topConfig.roi.top_left.x) &&
      _nh.getParam(topicPrefix + "/roi/top_left/y", topConfig.roi.top_left.y) &&
      _nh.getParam(topicPrefix + "/roi/bottom_right/x", topConfig.roi.bottom_right.x) &&
      _nh.getParam(topicPrefix + "/roi/bottom_right/y", topConfig.roi.bottom_right.y) &&
      _nh.getParam(topicPrefix + "/score_topic", topConfig.score_topic) &&
      _nh.getParam(topicPrefix + "/score_img_topic", topConfig.score_img_topic) &&
      _loadColorConfig(topicPrefix + "/desired_color_range/min", topConfig.desired_color_range.min) &&
      _loadColorConfig(topicPrefix + "/desired_color_range/max", topConfig.desired_color_range.max);
  }
  bool _loadTableDescConfig() {
    // Helper function to load geometry_msgs::Point's from the param server
    // Expects to have the sub-fields as x, y, z
    auto loadPoint = [this](const std::string& topic, geometry_msgs::Point& pt) {
		       return
			 _nh.getParam(topic + "/x", pt.x) &&
			 _nh.getParam(topic + "/y", pt.y) &&
			 _nh.getParam(topic + "/z", pt.z);
		     };

    return
      loadPoint("conveyor_sorting/table_desc/center", _config.table_desc.center) &&
      loadPoint("conveyor_sorting/table_desc/size", _config.table_desc.size);
  }
  bool _loadPileSpawnerConfig() {
    static const std::string topicPrefix{"conveyor_sorting/pile_spawner"};

    PileSpawnerConfig& pileConfig{_config.pile_spawner};
    return
      _nh.getParam(topicPrefix + "/pile_width", pileConfig.pile_width) &&
      _nh.getParam(topicPrefix + "/pile_length", pileConfig.pile_length) &&
      _nh.getParam(topicPrefix + "/pile_height_diff", pileConfig.pile_height_diff) &&
      _nh.getParam(topicPrefix + "/pile_items/min", pileConfig.pile_items.min) &&
      _nh.getParam(topicPrefix + "/pile_items/max", pileConfig.pile_items.max);
  }
  bool _loadSweeperConfig() {
    static const std::string topicPrefix{"conveyor_sorting/sweeper"};

    SweeperConfig& sweeperConfig{_config.sweeper};
    return
      _nh.getParam(topicPrefix + "/motion/width", sweeperConfig.motion.width) &&
      _nh.getParam(topicPrefix + "/motion/height", sweeperConfig.motion.height) &&
      _nh.getParam(topicPrefix + "/max_iter", sweeperConfig.max_iter) &&
      _nh.getParam(topicPrefix + "/use_ot", sweeperConfig.use_ot) &&
      _nh.getParam(topicPrefix + "/move", sweeperConfig.move) &&
      _nh.getParam(topicPrefix + "/seed", sweeperConfig.seed) &&
      _nh.getParam(topicPrefix + "/discrete_angles", sweeperConfig.discrete_angles) &&
      _nh.getParam(topicPrefix + "/sweep_topic", sweeperConfig.sweep_topic) &&
      _nh.getParam(topicPrefix + "/camera_intrinsics_topic", sweeperConfig.camera_intrinsics_topic) &&
      _nh.getParam(topicPrefix + "/camera_frame", sweeperConfig.camera_frame) &&
      _nh.getParam(topicPrefix + "/world_frame", sweeperConfig.world_frame) &&
      _nh.getParam(topicPrefix + "/ot_sweeper/samples", sweeperConfig.ot_sweeper.samples) &&
      _nh.getParam(topicPrefix + "/ot_sweeper/spread_area", sweeperConfig.ot_sweeper.spread_area);
  }
  bool _loadMotionConfig() {
    static const std::string topicPrefix("conveyor_sorting/motion_planner");

    MotionConfig& motionConfig{_config.motion_planner};
    return
      _nh.getParam(topicPrefix + "/initial_joints", motionConfig.initial_joints) &&
      _nh.getParam(topicPrefix + "/execute", motionConfig.execute);
  }
};

#endif
