#include "ros/ros.h"
#include "dynamixel_motors/dynamixel_node.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "hard_interface");
    ros::NodeHandle n;
    
    std::vector<int> ids = {20,21};
    std::vector<int> operating_mode = {3,3};
    std::vector<int> dir = {1,1};
    float protocol = 2.0;
    int baud  = 57600;
    std::vector<double> cur_limits = {1.0, 1.0};

    DynamixelNode DynNode("XM", ids, "/dev/ttyUSB0", dir, operating_mode, protocol, baud, cur_limits);

    ros::spin();


return 0;
}
