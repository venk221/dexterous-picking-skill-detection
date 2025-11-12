#include <string>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "conveyor_sorting/config.hpp"

cv_bridge::CvImagePtr latestImg{};
uint32_t latestSeq{0};

void recvTopMap(const sensor_msgs::Image& topographicalMap) {
  ROS_DEBUG_STREAM("Received topographical map image at " << topographicalMap.header.stamp);

  try {
    latestImg = cv_bridge::toCvCopy(topographicalMap);
  } catch(cv_bridge::Exception& e) {
    std::cerr << "Failed to convert image using cv_bridge: " << e.what() << '\n';
    return;
  }

  // Success, set the latest image sequence
  latestSeq = topographicalMap.header.seq;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "display_topmap_node");
  ros::NodeHandle nh{};

  ConfigLoader cfgLoader{nh};
  if(!cfgLoader.loadParams()) {
    ROS_FATAL("Failed to load params");
    return 1;
  }

  double resize{1.0};
  if(argc == 2) {
    resize = std::stod(std::string(argv[1]));
  }

  const TopographicalMapConfig& topConfig{cfgLoader.getConfig().topographical_map};

  ros::Subscriber topographicalMapSub{nh.subscribe(topConfig.topographical_map_topic, 1, recvTopMap)};

  // Compute a scale to convert topographical map heights to intensity values from 0.0 to 1.0
  const double scale{1.0 / topConfig.peak_distance};

  // Last processed sequence, used to know whether we should run or not
  uint32_t lastProcessedSeq{0};
  ros::Rate r{65};
  const double period{(1.0 / 65) * 1000}; // Period in ms for opencv waitKey
  ROS_INFO("Processing frames");
  while(ros::ok()) {
    ros::spinOnce();

    // We have a new frame to process
    if(latestSeq > lastProcessedSeq) {
      latestImg->image *= scale;
      cv::Mat scaledImg{};
      cv::resize(latestImg->image, scaledImg, cv::Size{}, resize, resize, cv::INTER_NEAREST);
      cv::imshow("topographical_map", scaledImg);
      cv::waitKey(period);

      lastProcessedSeq = latestSeq;
    }

    r.sleep();
  }

  return 0;
}
