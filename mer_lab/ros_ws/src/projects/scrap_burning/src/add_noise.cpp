#include <random>
#include <chrono>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "scrap_burning/config/config.hpp"

// 'Random' stuff
std::default_random_engine* generator;
std::normal_distribution<double>* dist;
std::uniform_real_distribution<double>* unif_dist;

ros::Publisher rgb_pub;
ros::Publisher dep_pub;
bool is_relative{false};
// Noise parameters
double mean, stddev, cutoff, left, right;

inline double clamp(const double& val, const double& bounds) {
  // Above bound
  if(val > bounds)
    return bounds;
  // Below bound
  else if(val < -bounds)
    return -bounds;

  return val;
}

void gaussian_callback(sensor_msgs::Image msg) {
  if(msg.encoding != "32FC1") {
    rgb_pub.publish(msg);
    return;
  }

  auto begin = ros::WallTime::now();
  cv_bridge::CvImagePtr cv;
  try {
    cv = cv_bridge::toCvCopy(msg, std::string("32FC1"));
  } catch(cv_bridge::Exception &e) {
    ROS_WARN_STREAM("Failed to convert image: " << e.what());
    return;
  }

  if(is_relative)
    for(int i = 0; i < cv->image.rows; ++i)
      for(int j = 0; j < cv->image.cols; ++j)
  	cv->image.at<float>(i, j) += clamp(cv->image.at<float>(i, j) * ((*dist)(*generator)), cutoff);
  else
    for(int i = 0; i < cv->image.rows; ++i)
      for(int j = 0; j < cv->image.cols; ++j)
  	cv->image.at<float>(i, j) += clamp((*dist)(*generator), cutoff);

  auto end = ros::WallTime::now();
  ROS_INFO_STREAM("Processing finished in " << (end - begin).toNSec() * 1e-9);
  dep_pub.publish(cv->toImageMsg());
}

void uniform_callback(const sensor_msgs::Image& msg) {
  if(msg.encoding != "32FC1") {
    rgb_pub.publish(msg);
    return;
  }

  auto begin = ros::WallTime::now();
  cv_bridge::CvImagePtr cv;
  try {
    cv = cv_bridge::toCvCopy(msg);
  } catch(cv_bridge::Exception &e) {
    ROS_WARN_STREAM("Failed to convert image: " << e.what());
    return;
  }

  if(is_relative)
    for(int i = 0; i < cv->image.rows; ++i)
      for(int j = 0; j < cv->image.cols; ++j)
	cv->image.at<float>(i, j) += (cv->image.at<float>(i, j) * ((*unif_dist)(*generator)));
  else
    for(int i = 0; i < cv->image.rows; ++i)
      for(int j = 0; j < cv->image.cols; ++j)
	cv->image.at<float>(i, j) += (*unif_dist)(*generator);
  
  auto end = ros::WallTime::now();
  ROS_INFO_STREAM("Processing finished in " << (end - begin).toNSec() * 1e-9);
  dep_pub.publish(cv->toImageMsg());
}

// Does nothing but republish the images
void copy_callback(const sensor_msgs::Image &img) {
  if(img.encoding != "32FC1")
    rgb_pub.publish(img);
  else
    dep_pub.publish(img);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "add_noise_node");
  ros::NodeHandle nh;

  ros::Subscriber img_sub;

  // Setup parameters
  generator = new std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
  scrap_burning::ScrapBurningConfig sbc(nh);
  sbc.loadParams();
  if(sbc.noise_type == "gaussian") {
    img_sub = nh.subscribe(sbc.image_topic, 1, gaussian_callback);
    dist = new std::normal_distribution<double>(sbc.mean, sbc.stddev);
    // Set the cutoff
    cutoff = sbc.cutoff;
  }
  else if(sbc.noise_type == "uniform") {
    img_sub = nh.subscribe(sbc.image_topic, 1, uniform_callback);
    unif_dist = new std::uniform_real_distribution<double>(sbc.left, sbc.right);
  }
  else if(sbc.noise_type == "none")
    img_sub = nh.subscribe(sbc.depth_topic, 1, copy_callback);
  else {
    ROS_ERROR("Unsupported noise type, please use only [gaussian|uniform]");
    return 2;
  }

  // Set whether it is relative or not
  is_relative = sbc.is_relative;

  // Advertise noisy depth/rgb topics
  dep_pub = nh.advertise<sensor_msgs::Image>(sbc.noise_depth_topic, 1);
  rgb_pub = nh.advertise<sensor_msgs::Image>(sbc.noise_rgb_topic, 1);

  ros::spin();

  return 0;
}
