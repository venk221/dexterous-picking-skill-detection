#include <fstream>
#include <string>
#include <iostream>
#include <chrono>

#include "ros/ros.h"
#include "encoderless_vs/control_points.h"
#include "std_msgs/Bool.h"

// Global start flag
bool start_flag = false;

void start_flag_callback(const std_msgs::Bool &msg){
    start_flag = msg.data;
}

int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "test_cp_node");
    ros::NodeHandle n;

    // Wait for services
    ros::service::waitForService("control_points_output", 1000);
    ros::service::waitForService("binary_image_output", 1000);

    // Initialize subscribers
    ros::Subscriber start_flag_sub = n.subscribe("/vsbot/control_flag",1,start_flag_callback);

    // Create Client object
    ros::ServiceClient cp_client = n.serviceClient<encoderless_vs::control_points>("control_points_output");
    
    // Create csv files
    std::ofstream timer("cp_srv_time.csv");

    // Add column names to files
    timer << "Control Points, Response Time"<<"\n";

    // Msg object for service Req and Res
    encoderless_vs::control_points cp_msg;
    
    cp_msg.request.input = 1;
    std::vector<float> cp_vec;

    ros::Rate r{30};
    while(ros::ok()){
        if(start_flag){
            // Time start
            auto st_time = std::chrono::high_resolution_clock::now();
            
            // Client call
            cp_client.call(cp_msg);
            cp_vec.clear();
            
            // Populate response
            for(int i = 0; i<6; i++){
            cp_vec.push_back(cp_msg.response.cp.data.at(i));
            }

            // Time end
            auto end_time = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time-st_time);
            
            // Write time & data to file
            
            // Print response
            for(std::vector<float>::iterator itr = cp_vec.begin(); itr != cp_vec.end(); ++itr){
                timer << *itr << " ";
            }
            timer << ", ";
            timer << duration.count()<<"\n";
        }
        ros::spinOnce();
        r.sleep();
    }
    timer.close();

    // Shutdown
    ros::spin();
    return 0;
}