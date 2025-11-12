// This node sends an initialization command to the origami robot
// The module is retracted to minimum length
// We then measure the length from base to tip

#include "ros/ros.h"
#include <std_msgs/Int32MultiArray.h>

#define MAX_PWM 1500

int main(int argc, char **argv){
    ros::init(argc, argv, "initialize module");
    ros::NodeHandle n;
    int num_actuators;
    n.getParam("origami_skeleton_vs/no_of_actuators", num_actuators);
    
    // Initialize publisher
    ros::Publisher vel_pub = n.advertise<std_msgs::Int32MultiArray>("origami_vs/velocity", 1);

    // Publish init vel
    std_msgs::Int32MultiArray init_vel;
    init_vel.layout.dim.push_back(std_msgs::MultiArrayDimension());
    init_vel.data.clear();
    
    for(int i=0; i<num_actuators; i++){
        init_vel.data.push_back(MAX_PWM);
    }
    
    // Waiting for publisher to be setup
    ros::Duration(1).sleep();

    // Refresh ROS
    ros::spinOnce();

    // Publish init command
    vel_pub.publish(init_vel);

    std::cout<<"Initialization command published"<<std::endl;

    ros::Duration(10).sleep();

    // Refresh ROS
    ros::spinOnce();

    std::cout<<"Shutting down"<<std::endl;
    return 0;
}