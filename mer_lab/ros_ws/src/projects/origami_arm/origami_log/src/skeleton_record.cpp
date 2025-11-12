#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "rosbag/bag.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/MultiArrayDimension.h"

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdlib>

rosbag::Bag bag;

std_msgs::Float64MultiArray error;
std_msgs::Float64MultiArray features;
std_msgs::Int32MultiArray velocity;
std_msgs::Float64 model_error;
std_msgs::Float64MultiArray aruco_pose;

sensor_msgs::Image final_img;
sensor_msgs::Image raw_img;
sensor_msgs::Image curve_img;
sensor_msgs::Image skeleton;

bool end_flag = false;
bool record_flag = false;


void velocityWriter(const std_msgs::Int32MultiArray &msg){
    velocity = msg;
}

void errorWriter(const std_msgs::Float64MultiArray &msg){
    error = msg;
}

void modelErrorWriter(const std_msgs::Float64 &msg){
    model_error = msg;
}

void featureWriter(const std_msgs::Float64MultiArray &msg){
    features = msg;
}

void skeletonWriter(const sensor_msgs::Image &img){
    skeleton = img;
}

void imgWriter(const sensor_msgs::Image &img){
    final_img = img;
}

void rawimgWriter(const sensor_msgs::Image &img){
    raw_img = img;
}

void curveimgWriter(const sensor_msgs::Image &img){
    curve_img = img;
}

void endFlagWriter(const std_msgs::Bool &msg){
    end_flag = msg.data;
    // std::cout<<"end flag: "<<end_flag<<std::endl;
}

void recordFlagCallback(const std_msgs::Int32 &msg){
    record_flag = msg.data;
}

void arucoLog(const std_msgs::Float64MultiArray &msg){
    aruco_pose = msg;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "origami_logger");
    ros::NodeHandle n;

    // Get home path
    std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";
    
    // current system date/time
    time_t now = time(0);
    std::string time_str = std::string(ctime(&now));
    std::replace(time_str.begin(), time_str.end(),':','-');
    std::replace(time_str.begin(), time_str.end(), ' ', '_');
    // std::cout<<"TIMESTAMP: "<<time_str<<std::endl;
    // Bag file name with current time
    std::string bagfile = HOME+"/.ros/"+time_str.substr(0, time_str.length()-1) + ".bag";

    // Open bag file
    bag.open(bagfile, rosbag::bagmode::Write);

    // Subscribers
    ros::Subscriber vel_log = n.subscribe("origami_vs/velocity", 1, velocityWriter);
    ros::Subscriber error_log = n.subscribe("origami_vs/error", 1, errorWriter);
    ros::Subscriber end_log = n.subscribe("origami_vs/end_flag", 1, endFlagWriter);
    ros::Subscriber model_error_log = n.subscribe("origami_vs/model_error", 1, modelErrorWriter);
    ros::Subscriber img_log = n.subscribe("origami_vs/skeleton", 1, imgWriter);
    ros::Subscriber raw_img_log = n.subscribe("camera/color/image_raw",1, rawimgWriter);
    ros::Subscriber curve_img_log = n.subscribe("origami_vs/curve_image",1,curveimgWriter);
    ros::Subscriber skeleton_log = n.subscribe("origami_vs/binary_image", 1, skeletonWriter);
    // ros::Subscriber feature_log = n.subscribe("origami_vs/control_points", 1, featureWriter);
    ros::Subscriber feature_log = n.subscribe("origami_vs/feature_pub", 1, featureWriter);
    ros::Subscriber start_record = n.subscribe("origami_vs/start_record", 1, recordFlagCallback);
    ros::Subscriber aruco_log = n.subscribe("origami_vs/aruco/pose", 1, arucoLog);

    // Setup ros rate
    float rate = 0;
    n.getParam("origami_skeleton_vs/control_rate", rate);
    ros::Rate r{rate};

    // wait until experiment starts
    while(record_flag == 0){
        ros::spinOnce();
        // ros::Duration(1).sleep();
        // std::cout<<"Waiting for exp start"<<std::endl;
    }

    // rate loop to write latest data to bag
    while(!end_flag){
        ros::Time t = ros::Time::now();
        // std::cout<<"writing to bag"<<std::endl;
        // Write to bag
        bag.write("features", t, features);
        bag.write("error", t, error);
        bag.write("model_error", t, model_error);
        bag.write("velocity", t, velocity);
        bag.write("skeleton", t, skeleton);
        bag.write("rgb_image", t, final_img);
        bag.write("raw_image", t, raw_img);
        bag.write("curve_image", t, curve_img);
        bag.write("aruco_pose", t, aruco_pose);

        // ros rate
        ros::spinOnce();
        r.sleep();
    }
    std::cout<<"Attempting to close bag"<<std::endl;
    if(end_flag){
        // std::cout<<"Inside if"<<std::endl;
        bag.close();
        std::cout<<"Bag closed"<<std::endl;
        ros::shutdown();
    }

    ros::spin();
    return 0;
}
