// This node sends an initialization command to the origami robot
// The module is retracted to minimum length
// We then measure the length from base to tip

#include "ros/ros.h"
#include <std_msgs/UInt8MultiArray.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "initialize module");
    ros::NodeHandle n;

    // Read params from YAML
    int max_length_hi;
    n.getParam("origami_vs/init_max_length_hi", max_length_hi);
    
    int max_length_lo;
    n.getParam("origami_vs/init_max_length_lo", max_length_lo);

    int min_length_hi;
    n.getParam("origami_vs/init_min_length_hi", min_length_hi);
    
    int min_length_lo;
    n.getParam("origami_vs/init_min_length_lo", min_length_lo);

    // Initialize publisher
    ros::Publisher vel_pub = n.advertise<std_msgs::UInt8MultiArray>("origami_vs/velocity", 1);

    std_msgs::UInt8MultiArray init_cmd;
    init_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    init_cmd.layout.dim[0].size = 7;
    init_cmd.layout.dim[0].stride = 1;
    init_cmd.layout.dim[0].label="init_cmd";
    init_cmd.data.clear();

    init_cmd.data.push_back(max_length_hi);
    init_cmd.data.push_back(max_length_lo);

    init_cmd.data.push_back(min_length_hi);
    init_cmd.data.push_back(min_length_lo);

    init_cmd.data.push_back(0);   // Empty Byte
    init_cmd.data.push_back(0);   // Empty Byte
    init_cmd.data.push_back(0);   // Command type, "0: initialization"

    // Waiting for publisher to be setup
    ros::Duration(1).sleep();

    // Refresh ROS
    ros::spinOnce();

    // Publish init command
    vel_pub.publish(init_cmd);

    std::cout<<"Initialization command published"<<std::endl;

    ros::Duration(10).sleep();

    std::cout<<"Shutting down"<<std::endl;
    return 0;
}