#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/photo.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "conveyor_sorting/config.hpp"

static char requiredEncoding[]{"32FC1"};
static int divisions{3};
static double groundDist{0.4};
static double peakDist{0.5};
static ROI roi{};

static ros::Publisher topPub;

void imgCallback(sensor_msgs::ImageConstPtr img) {
  // This contains the mask since we don't need to re-do it for every image
  static cv::Mat mask{};

  // Skip if the image does not have the correct encoding
  if(img->encoding != requiredEncoding)
    return;

  cv_bridge::CvImagePtr cvImgPtr{};
  try {
    cvImgPtr = cv_bridge::toCvCopy(img, requiredEncoding);
  } catch(cv_bridge::Exception& e) {
    std::cerr << "Failed to convert image using cv_bridge: " << e.what() << '\n';
    return;
  }

  // Set all pixels outside the ROI to the table height
  // Construct a mask around the ROI if it isn't available yet
  if(mask.empty()) {
    mask = cv::Mat::zeros(cvImgPtr->image.rows, cvImgPtr->image.cols, CV_32FC1);
    mask(cv::Rect(roi.top_left.x, roi.top_left.y,
                  roi.bottom_right.x - roi.top_left.x,
                  roi.bottom_right.y - roi.top_left.y)) = 1.0;
  }

  // Infill NaNs
  // Temporary hack to at least have valid values, just set these to 0
  cvImgPtr->image.setTo(groundDist, cvImgPtr->image <= 0.0);
  // cv::Mat negMask(cvImgPtr->image <= 0.0);
  // cv::inpaint(cvImgPtr->image, negMask, cvImgPtr->image, 10.0, cv::INPAINT_NS);
  // // Resize image to a small size
  // cv::Mat milImg(cvImgPtr->image.rows, cvImgPtr->image.cols, CV_16UC1);
  // cvImgPtr->image.convertTo(milImg, CV_16UC1, 1000);
  // cv::Mat smallMilImg;
  // cv::resize(milImg, smallMilImg, cv::Size(), 0.2, 0.2);
  // cv::Mat inpaintTemp{};
  // cv::inpaint(smallMilImg, smallMilImg == 1000, inpaintTemp, 5.0, cv::INPAINT_TELEA);
  // cv::Mat resizedInpaintTemp{};
  // cv::resize(inpaintTemp, resizedInpaintTemp, cvImgPtr->image.size());
  // resizedInpaintTemp.copyTo(milImg, (milImg == 1000));
  // milImg.convertTo(cvImgPtr->image, CV_32FC1, 1.0 / 1000.0)

  // First, subtract the ground_distance from each pixel and flip the sign, then clamp
  cvImgPtr->image -= groundDist;
  cvImgPtr->image *= -1;
  cvImgPtr->image.setTo(0.0, (cvImgPtr->image < 0.0));

  // Difference between each level
  double diff{peakDist / divisions};
  double prevContourVal{0.0001};
  for(double i = diff; i < peakDist; i += diff) {
    cvImgPtr->image.setTo(i, (cvImgPtr->image > prevContourVal) & (cvImgPtr->image <= i));

    prevContourVal = i;
  }

  // Mask out the image
  cvImgPtr->image = cvImgPtr->image.mul(mask);

  topPub.publish(cvImgPtr->toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "topographical_mapper_node");
  ros::NodeHandle nh{};

  ConfigLoader cfgLoader{nh};
  if(!cfgLoader.loadParams()) {
    std::cerr << "Failed to load all parameters\n";
    return 1;
  }

  // Setup topographical map constants
  groundDist = cfgLoader.getConfig().topographical_map.camera_table_offset;
  divisions = cfgLoader.getConfig().topographical_map.division_count;
  peakDist = cfgLoader.getConfig().topographical_map.peak_distance;
  roi = cfgLoader.getConfig().topographical_map.roi;
  ROS_INFO_STREAM("Creating topographical maps with " << divisions << " slices until " << peakDist << " distance");

  // Setup subscriber and publisher
  std::cout << "Listening to image topic " << cfgLoader.getConfig().topographical_map.input_depth_topic << "\n";
  ros::Subscriber imgSub{nh.subscribe(cfgLoader.getConfig().topographical_map.input_depth_topic, 1, imgCallback)};
  topPub = nh.advertise<sensor_msgs::Image>(cfgLoader.getConfig().topographical_map.topographical_map_topic, 1);

  ros::spin();

  return 0;
}
