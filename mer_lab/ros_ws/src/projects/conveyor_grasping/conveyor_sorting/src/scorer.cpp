#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <qnd/cam.hpp>

#include "conveyor_sorting/config.hpp"

static constexpr bool publishImage{true};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scorer_node");
  ros::NodeHandle nh{};

  ConfigLoader cfgLoader{nh};
  if(!cfgLoader.loadParams()) {
    ROS_FATAL_STREAM("Failed to load one or more params");
    return 1;
  }

  ros::Publisher scorePub{nh.advertise<std_msgs::Float64>(cfgLoader.getConfig().topographical_map.score_topic, 1)};
  ros::Publisher imgPub{nh.advertise<sensor_msgs::Image>("scored_image", 1)};

  double backgroundHeight{cfgLoader.getConfig().topographical_map.camera_table_offset};
  Color cMin{cfgLoader.getConfig().topographical_map.desired_color_range.min};
  Color cMax{cfgLoader.getConfig().topographical_map.desired_color_range.max};
  cMin.h /= 2;
  cMax.h /= 2;
  cMin.s /= (100.0 / 255);
  cMax.s /= (100.0 / 255);
  cMin.v /= (100.0 / 255);
  cMax.v /= (100.0 / 255);

  ros::Rate r{2};
  qnd::RosCam rgbCam{nh,
		     cfgLoader.getConfig().topographical_map.input_color_topic,
		     cfgLoader.getConfig().sweeper.camera_intrinsics_topic,
		     cfgLoader.getConfig().topographical_map.input_color_encoding};
  qnd::RosCam depCam{nh,
		     cfgLoader.getConfig().topographical_map.input_depth_topic,
		     cfgLoader.getConfig().sweeper.camera_intrinsics_topic,
		     "32FC1"};
  ROS_DEBUG("Waiting for image on RGB");
  rgbCam.waitForImage();
  ROS_DEBUG("Waiting for image on DEP");
  depCam.waitForImage();
  while(ros::ok()) {
    cv::Mat rgbImg(rgbCam.getLatestImgCV());
    cv::Mat depImg(depCam.getLatestImgCV());

    // Check how far spread the items are in the depth image
    // First do background subtraction
    depImg -= backgroundHeight;
    depImg *= -1;
    depImg.setTo(0.0, (depImg < 0.0));

    // Now flag the pixels > 0.0
    depImg.setTo(1.0, (depImg >= 0.00001));

    // Compute the weights of each pixel based on the color
    cv::Mat weightImg{};
    // Convert the color to hsv
    cv::Mat hsvImg{};
    cvtColor(rgbImg, hsvImg, CV_BGR2HSV);
    // Filter by hsv
    cv::inRange(hsvImg,
		cv::Scalar(cMin.h, cMin.s, cMin.v),
		cv::Scalar(cMax.h, cMax.s, cMax.v),
		weightImg);
    weightImg /= 255;
    weightImg += 1;
    // Comment this out to take color into account
    weightImg *= 0;
    weightImg += 1;

    // Remove any weights for now
    weightImg = cv::Scalar(1);

    // Multiply the visible pixels by the weight and sum
    cv::Mat weightedScoreImg{depImg.rows, depImg.cols, CV_8UC1};
    cv::multiply(depImg, weightImg, weightedScoreImg, 1, CV_8UC1);

    double score = cv::sum(weightedScoreImg)[0];

    // Publish score
    std_msgs::Float64 scoreMsg;
    scoreMsg.data = score;
    scorePub.publish(scoreMsg);

    // If necessary, publish the image as well
    if(publishImage) {
      // Color the image for display
      cv::Mat coloredImage(weightedScoreImg);
      coloredImage *= 120;
      imgPub.publish(qnd::fromCV(coloredImage, ros::Time::now(), "world", "8UC1"));
    }

    ros::spinOnce();

    r.sleep();
  }
}
