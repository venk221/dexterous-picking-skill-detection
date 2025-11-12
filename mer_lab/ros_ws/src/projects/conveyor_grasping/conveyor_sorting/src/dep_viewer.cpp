#include <string>

#include <ros/ros.h>

#include <conveyor_sorting/config.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <qnd/cam.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "dep_viewer_node");
  ros::NodeHandle nh{};

  double scale{1.0};
  if(argc == 2) {
    scale = std::stod(std::string(argv[1]));
  }

  ROS_INFO_STREAM("Scaling image by " << scale);

  ConfigLoader cfgLoader{nh};
  cfgLoader.loadParams();

  std::string imgTopic(cfgLoader.getConfig().topographical_map.input_depth_topic);
  std::string intTopic(cfgLoader.getConfig().sweeper.camera_intrinsics_topic);

  qnd::RosCam cam{nh, imgTopic, intTopic, "32FC1"};

  cam.waitForImage();

  ros::Rate r{30};
  while(ros::ok()) {
    r.sleep();

    cv::Mat img(cam.getLatestImgCV());
    img /= cfgLoader.getConfig().topographical_map.camera_table_offset;

    cv::Mat resizedImg{};
    cv::resize(img, resizedImg, cv::Size{}, scale, scale, cv::INTER_NEAREST);

    cv::imshow("depth_image", resizedImg);

    cv::waitKey(1.0 / 30 * 1000);

    ros::spinOnce();
  }

  return 0;
}
