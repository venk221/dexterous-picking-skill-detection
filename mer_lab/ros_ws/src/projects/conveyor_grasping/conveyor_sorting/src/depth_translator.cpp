#include <ros/ros.h>
#include <qnd/cam.hpp>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include "conveyor_sorting/config.hpp"

static constexpr double rate{15.0};
static constexpr char sourceEncoding[]{"16UC1"};
static constexpr auto sourceCVEncoding{CV_32FC1};
static constexpr char targetEncoding[]{"32FC1"};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_translator_node");
    ros::NodeHandle nh{};

    ConfigLoader cfgLoader{nh};
    if(!cfgLoader.loadParams()) {
        ROS_FATAL("Failed to load params from ConfigLoader");
        return 1;
    }

    // Subscriber
    qnd::RosCam cam{nh, cfgLoader.getConfig().topographical_map.raw_depth_topic,
                    cfgLoader.getConfig().sweeper.camera_intrinsics_topic,
                    sourceEncoding};
    // Publisher
    ros::Publisher camPub = nh.advertise<sensor_msgs::Image>(cfgLoader.getConfig().topographical_map.input_depth_topic, 1);
    // Rate
    ros::Rate r(rate);

    cam.waitForImage();

    while(ros::ok()) {
        ros::spinOnce();

        cv::Mat latestImg(cam.getLatestImgCV());
        cv::Mat convertedImg(latestImg.rows, latestImg.cols, sourceCVEncoding);
        latestImg.convertTo(convertedImg, sourceCVEncoding, 1.0 / 1000.0);

        camPub.publish(qnd::fromCV(convertedImg, cam.getLatestImgTime(), cam.getLatestImgFrame(), targetEncoding));

        r.sleep();
    }

    return 0;
}