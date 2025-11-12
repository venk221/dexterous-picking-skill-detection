// This node records raw experiment data to a bag file
// The bag file name is the file creation timestamp
// The bag file is located in /.ros/

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
// #include "std_msgs/UInt8MultiArray.h"
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

std_msgs::Float64MultiArray ee_pose;

std_msgs::Float64MultiArray velocity;

std_msgs::Float64 model_error;

sensor_msgs::Image img;

bool end_flag = false;
bool record_flag = false;

void modelErrorWriter(const std_msgs::Float64 &msg){
    model_error = msg;
}

void errorWriter(const std_msgs::Float64MultiArray &msg){
    // std::vector<double> error_vec = msg.data;
    // error.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // error.data.clear();
    // error.data.insert(error.data.end(), error_vec.begin(), error_vec.end());
    error = msg;
}

void eePoseWriter(const std_msgs::Float64MultiArray &msg){
    ee_pose = msg;
}

void velocityWriter(const std_msgs::Float64MultiArray &msg){
    // std::vector<uint8_t> vel_vec = msg.data;
    // for(std::vector<uint8_t>::iterator itr=vel_vec.begin(); itr != vel_vec.end(); ++itr){
    //     std::cout<<*itr;
    // }
    // std::cout<<std::endl;
    // velocity.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // velocity.data.clear();
    // velocity.data.insert(velocity.data.end(), vel_vec.begin(), vel_vec.end());
    
    velocity = msg;

}

void rawImgWriter(const sensor_msgs::Image &msg){
    img = msg;
}

void endFlagCheck(const std_msgs::Bool &msg){
    end_flag = msg.data;
}

void recordFlag(const std_msgs::Bool &msg){
    record_flag = msg.data;
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
    ros::Subscriber vel_sub = n.subscribe("origami_vs/velocity_log", 1, velocityWriter);
    ros::Subscriber ee_pose_sub = n.subscribe("origami_vs/aruco/pose", 1, eePoseWriter);
    ros::Subscriber error_sub = n.subscribe("origami_vs/error", 1, errorWriter);
    ros::Subscriber rawImages = n.subscribe("origami_vs/aruco/result", 1, rawImgWriter);
    ros::Subscriber endFlag = n.subscribe("origami_vs/end_flag", 1, endFlagCheck);
    ros::Subscriber startRecord = n.subscribe("origami_vs/start_record", 1, recordFlag);
    ros::Subscriber modelError = n.subscribe("origami_vs/model_error", 1, modelErrorWriter);

    // Setup ros rate
    float rate = 0;
    n.getParam("origami_vs/control_rate", rate);
    ros::Rate r{rate};

    // wait until experiment starts
    while(!record_flag){
        ros::spinOnce();
        ros::Duration(1).sleep();
        // std::cout<<"Waiting for exp start"<<std::endl;
    }
    // rate loop to write latest data to bag
    while (!end_flag){
        ros::Time t = ros::Time::now();
        
        // Write to bag
        
        bag.write("ee_pose",t, ee_pose);
        bag.write("error",t, error);
        bag.write("raw_img",t, img);
        bag.write("velocity",t, velocity);
        bag.write("model_error", t, model_error);

        // ros rate
        ros::spinOnce();
        r.sleep();
    }
    
    if(end_flag){
        bag.close();
        ros::shutdown();
    }

    ros::spin();
    return 0;
}