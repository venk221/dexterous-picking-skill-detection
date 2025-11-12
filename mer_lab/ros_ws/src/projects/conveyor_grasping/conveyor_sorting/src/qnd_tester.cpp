#include <ros/ros.h>

#include <qnd/cam.hpp>
#include <qnd/vis.hpp>
#include <qnd/geom.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "qnd_tester_node");
  ros::NodeHandle nh{};

  // Create a cam which listens to depth images
  qnd::RosCam cam{nh, "/camera1/depth/image_raw", "/camera1/depth/camera_info", "32FC1"};

  ros::Rate r{10};

  // Wait for our first image
  cam.waitForImage(r);

  // Initialize visualization
  qnd::Visualizer vis{nh};

  while(ros::ok()) {
    cv::Mat latest{cam.getLatestImgCV()};

    // Project the corners of the matrix
    const std::string& frame = cam.getLatestImg().header.frame_id;
    vis.visPoint(0, cam.fromImgCoord(qnd::createPt(0.0, 0.0,
						   latest.at<float>(0, 0))), frame);
    vis.visPoint(1, cam.fromImgCoord(qnd::createPt(latest.rows - 1, 0.0,
						   latest.at<float>(latest.rows - 1, 0))), frame);
    vis.visPoint(2, cam.fromImgCoord(qnd::createPt(0.0, latest.cols - 1,
						   latest.at<float>(0, latest.cols - 1))), frame);
    vis.visPoint(3, cam.fromImgCoord(qnd::createPt(latest.rows - 1, latest.cols - 1,
						   latest.at<float>(latest.rows - 1, latest.cols - 1))), frame);
    ros::spinOnce();
    r.sleep();
  }
}
