#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"


std::vector<int> v;
int num_actuators = 0;
std_msgs::Int32MultiArray formatted_velocity_msg;
ros::Publisher velocity_pub;


void velCallback(const std_msgs::Int32MultiArray &msg){
    v.clear();
    for(int i=0;i<num_actuators;i++){
        v.push_back(msg.data.at(i));
    }

    // for(std::vector<int>::iterator it = v.begin(); it!=v.end(); ++it){
    //     std::cout<< *it<<std::endl;
    // }

    formatted_velocity_msg.data.clear();
        // std::cout<<"creating vel msg"<<std::endl;
        // padding

        // Module 0
        for(int i=0; i<4; i++){
            formatted_velocity_msg.data.push_back(0);
        }
        
        // Module 1
        for(int i=0; i<4; i++){
            formatted_velocity_msg.data.push_back(0);
        }

        // Module 2
        formatted_velocity_msg.data.push_back(0);
        for(int i=0; i<2; i++){
            formatted_velocity_msg.data.push_back(v[i]);
        }
        formatted_velocity_msg.data.push_back(v[1]);

        // Module 3
        formatted_velocity_msg.data.push_back(0);
        for(int i=2; i<4; i++){
            formatted_velocity_msg.data.push_back(v[i]);
        }
        formatted_velocity_msg.data.push_back(v[3]);

        // Modules 4-7
        for(int i=0; i<16; i++){
            formatted_velocity_msg.data.push_back(0);
        }

        velocity_pub.publish(formatted_velocity_msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_test");
    ros::NodeHandle n;
    
    n.getParam("origami_skeleton_vs/no_of_actuators", num_actuators);

    ros::Subscriber velocity_sub = n.subscribe("origami_vs/velocity", 1, velCallback);
    velocity_pub = n.advertise<std_msgs::Int32MultiArray>("origami_vs/OMMD_control_input", 1);


    ros::spin();
    return 0;
}