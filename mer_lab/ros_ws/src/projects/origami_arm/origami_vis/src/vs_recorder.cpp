// This node records to csv the following tiems
// 1. Published actuator velocities
// 2. Marker positions
// 3. Error in image space

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
// #include "std_msgs/UInt8MultiArray.h"

#include <iostream>
#include <fstream>
#include <string>


void errorWriter(const std_msgs::Float64MultiArray &msg){
    std::ofstream writeError("error.csv", std::ios::app);     // Create file if it does not exist, otherwise append to file

    // Convert data to c++ vector
    std::vector<double> error = msg.data;

    // Each element in vector is entered to a separate column
    for(std::vector<double>::iterator itr = error.begin(); itr!= error.end(); ++itr){
        writeError <<*itr <<"," ;
    }

    // Move to next line and close file
    writeError <<"\n";
    writeError.close();
}


void eePoseWriter(const std_msgs::Float64MultiArray  &msg){
    std::ofstream writeEEPose("ee_pose.csv", std::ios::app);     // Create file if it does not exist, otherwise append to file

    // Convert data to c++ vector
    std::vector<double> ee_pose = msg.data;

    // Each element in vector is entered to a separate column
    for(std::vector<double>::iterator itr = ee_pose.begin(); itr!= ee_pose.end(); ++itr){
        writeEEPose <<*itr <<"," ;
    }

    // Move to next line and close file
    writeEEPose <<"\n";
    writeEEPose.close();
}   


void velocityWriter(const std_msgs::Float64MultiArray &msg){
    std::ofstream writeVelocity("velocity.csv", std::ios::app);  // Create file if it does not exist, otherwise append to file
    
    // Convert data to c++ vector
    std::vector<double> velocity = msg.data;

    // Each element in vector is entered to a separate column
    for(std::vector<double>::iterator itr = velocity.begin(); itr!=velocity.end(); ++itr){
        std::cout<< "vel: " << *itr <<"'";
        writeVelocity <<*itr <<",";
    }
    std::cout<<std::endl;

    // Move to next line and close file
    writeVelocity <<"\n";
    writeVelocity.close();
}


int main(int argc, char ** argv){
    ros::init(argc, argv, "data recorder");
    ros::NodeHandle n;

    // Subscribe to data
    ros::Subscriber vel = n.subscribe("origami_vs/velocity_log", 1, velocityWriter);
    ros::Subscriber ee_pose = n.subscribe("origami_vs/aruco/pose", 1, eePoseWriter);
    ros::Subscriber error = n.subscribe("origami_vs/error", 1, errorWriter);

    

    

    ros::spin();
    return 0;
}