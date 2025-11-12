#include <stdlib.h>
#include <iostream>
#include <random>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

#include <Eigen/Geometry>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include "scrap_burning/config/config.hpp"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef pcl::PointCloud<PointType>::Ptr CloudPtr;

enum ImageState{ None=0, LoadedRGB=1, LoadedDepth=2, LoadedAll=3 };
ImageState operator|(ImageState a, ImageState b) {
  return static_cast<ImageState>(static_cast<int>(a) | static_cast<int>(b));
}
ImageState state = None;
cv::Mat rgbImg;
cv::Mat depthImg;
std::string imgFrame;

template <typename T>
class MovingAverage {
public:
  MovingAverage(std::size_t count)
    : _measurements(0), _avg(), _window(count) {}

  void addMeasurement(const T &measurement) {
    _window[_measurements++ % _window.size()] = measurement;
    // Update average
    _avg = std::accumulate(_window.begin(), _window.end(), T()) / std::min(_window.size(), _measurements);
  }

  T getAvg() const { return _avg; }
private:
  std::size_t _measurements;
  T _avg;
  std::vector<T> _window;
};

CloudPtr convertCloud(cv::Mat depthImg, cv::Mat rgbImg, const sensor_msgs::CameraInfo &info) {
  CloudPtr ret(new CloudType());

  // Set dimensions
  ret->is_dense = true;
  ret->height = depthImg.rows;
  ret->width  = depthImg.cols;
  ret->points.reserve(depthImg.rows * depthImg.cols);

  // Create focal lengths
  float fx = 1.0f / info.K[0];
  float fy = 1.0f / info.K[4];

  // Begin projecting the points
  for(int i = 0; i < depthImg.rows; ++i) {
    for(int j = 0; j < depthImg.cols; ++j) {
      float depth = depthImg.at<float>(i, j);
      cv::Vec3b col = rgbImg.at<cv::Vec3b>(i, j);
      if(std::isnan(depth)) {
  // We sometimes have ambiguous function overload resolving, so just cast to uint8_t to enforce
  // the r, g, b constructor
	PointType pt(static_cast<uint8_t>(0), 0, 0);
	pt.x = 0;
	pt.y = 0;
	pt.z = 0;
	ret->push_back(pt);
	
      }
      else {
	PointType pt(col[2], col[1], col[0]);
	pt.x = (j - info.K[2]) * depth * fy;
	pt.y = (i - info.K[5]) * depth * fx;
	pt.z = depth;
	ret->push_back(pt);
      }
    }
  }

  return ret;
}

void addNoise(cv::Mat img, double stddev) {
  static std::random_device rd{};
  static std::mt19937 gen{rd()};
  static std::normal_distribution<> d{0, stddev};

  for(int i = 0; i < img.rows; ++i)
    for(int j = 0; j < img.cols; ++j)
      img.at<float>(i, j) += d(gen);
}

void imageCallback(sensor_msgs::Image msg) {
  cv_bridge::CvImagePtr cv;
  try {
    cv = cv_bridge::toCvCopy(msg);
  } catch(cv_bridge::Exception &e) {
    ROS_WARN_STREAM("Failed to convert image: " << e.what());
    return;
  }
  
  if(msg.encoding == "32FC1") {
    depthImg = cv->image;
    state = state | LoadedDepth;
  }
  else {
    rgbImg = cv->image;
    state = state | LoadedRGB;
  }

  imgFrame = msg.header.frame_id;
}

void waitForImage() {
  state = None;
  while(state != LoadedAll)
    ros::spinOnce();
}

double computeDelta(const cv::Mat a, const cv::Mat b) {
  double sumDiff = 0.0;

  for(int i = 0; i < a.rows; ++i) {
    for(int j = 0; j < a.cols; ++j) {
      if(std::isnan(a.at<float>(i, j)) || std::isnan(b.at<float>(i, j)))
	continue;
      sumDiff += abs(a.at<float>(i, j) - b.at<float>(i, j));
    }
  }

  return sumDiff / (a.rows * a.cols);
}

CloudPtr transformCloud(const geometry_msgs::TransformStamped &trans, CloudPtr cloud) {
  CloudPtr ret(new CloudType());

  // Create an eigen transform from the geometry_msgs transform
  Eigen::Affine3f eigenTrans(Eigen::Affine3f::Identity());
  eigenTrans.linear() = Eigen::Quaternion<float>(trans.transform.rotation.w,
						 trans.transform.rotation.x,
						 trans.transform.rotation.y,
						 trans.transform.rotation.z).toRotationMatrix();
  eigenTrans.translation() = Eigen::Vector3f(trans.transform.translation.x,
					     trans.transform.translation.y,
					     trans.transform.translation.z);
  pcl::transformPointCloud(*cloud, *ret, eigenTrans);

  return ret;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ground_truth_generator_node");
  ros::NodeHandle nh;

  // Load params
  scrap_burning::ScrapBurningConfig config(nh);
  config.loadParams();

  // Setup tf2 buffer/listener
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tfListener(buffer);

  // We are interested in two topics, the image topic, and the camera info topic
  std::string depthTopic(config.depth_topic.substr(0, config.depth_topic.find("points")) + "image_raw");
  std::string infoTopic(config.depth_topic.substr(0, config.depth_topic.find("points")) + "camera_info");

  // Get camera info (only need this once)
  sensor_msgs::CameraInfoConstPtr camInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(infoTopic);
  // Create image subcriber
  ros::Subscriber imageSub = nh.subscribe(depthTopic, 1, imageCallback);

  // Create PCL Visualization window
  pcl::visualization::PCLVisualizer vis;
  // Specify the two point clouds we have
  CloudPtr registeredCloud(new CloudType());
  CloudPtr currentCloud(new CloudType());
  vis.addPointCloud(registeredCloud, "Registered Cloud");
  vis.addPointCloud(currentCloud, "Current Cloud");

  // Begin main loop
  bool run = true;
  while(run) {
    // Ask user to move the robot to the next position
    std::cout << "Move robot into position.  Done? (Y/n): ";
    char c = 'n';
    while(c == 'n')
      std::cin >> c;

    // load the first image
    waitForImage();
    // This is the latest average
    cv::Mat avgImg = depthImg.clone();
    // This is the previous average
    cv::Mat prevAvg;

    // Get the new transform to the base frame
    geometry_msgs::TransformStamped trans;
    bool found = false;
    for(int i = 0; i < 3 && !found; ++i) {
      try {
	trans = buffer.lookupTransform("world", imgFrame, ros::Time(0));
	found = true;
      } catch(tf2::TransformException &ex) {
	ROS_WARN_STREAM("Failed to get transform: " << ex.what());
	ros::Duration(2.5).sleep();
      }
    }
    if(!found) {
      ROS_ERROR("Failed to find transform from camera image frame");
      return 1;
    }

    // Number of images obtained in total, used to compute the average
    int imgCount = 1;
    // This is the moving average of the deltas of the last N frames
    MovingAverage<double> mav(5);

    ROS_INFO("Loaded first image, beginning loop");
    while(true) {
      waitForImage();

      // Compute new and previous average
      ++imgCount;
      prevAvg = avgImg.clone();
      avgImg = ((avgImg * (imgCount - 1)) + depthImg) / imgCount;

      // Update moving average
      mav.addMeasurement(computeDelta(prevAvg, avgImg));
      ROS_INFO_STREAM("Average delta: " << mav.getAvg());
      if(mav.getAvg() <= 0.00001) break;

      // Project mat, update view and UI
      currentCloud = convertCloud(avgImg, rgbImg, *camInfo);
      currentCloud = transformCloud(trans, currentCloud);
      vis.updatePointCloud(currentCloud, pcl::visualization::PointCloudColorHandlerRGBField<PointType>(currentCloud), "Current Cloud");
      vis.spinOnce();
    }

    // Abort this insertion?
    std::cout << "Retry? (N/y): ";
    char abort;
    std::cin >> abort;
    if(abort == 'y') continue;

    // Register the new cloud
    registeredCloud->insert(registeredCloud->end(), currentCloud->begin(), currentCloud->end());
    vis.updatePointCloud(registeredCloud, pcl::visualization::PointCloudColorHandlerRGBField<PointType>(registeredCloud), "Registered Cloud");

    // Check if the user wants to continue
    std::cout << "Continue? (Y/n): ";
    char cont;
    std::cin >> cont;
    run = (cont != 'n');
  }

  // Once done, we save the registered clouds to a file
  pcl::io::savePCDFileASCII("/tmp/ground_truth_cloud.pcd", *registeredCloud);

  return 0;
}
