#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

std::vector<float> joint_velocity;
int no_of_actuators = 0;
int rate = 0;
bool end_flag = false;
std_msgs::Float64 v;

ros::Publisher j1_vel_pub;
ros::Publisher j2_vel_pub;

void getFlag(const std_msgs::Bool &msg){
    end_flag = msg.data;
}

void getVelocity(const std_msgs::Float64MultiArray &msg){
    joint_velocity.clear();
    for(int i=0; i<no_of_actuators; i++){
        joint_velocity.push_back(msg.data.at(i));
    }
    v.data = joint_velocity[0];
    j1_vel_pub.publish(v);
    
    v.data = joint_velocity[1];
    j2_vel_pub.publish(v);

}

int main(int argc, char **argv){
    
    // Init ROS node
    ros::init(argc, argv, "hardware_api");
    ros::NodeHandle n;

    // Read configuration parameters
    n.getParam("two_link_vs/no_of_actuators", no_of_actuators);
    n.getParam("two_link_vs/control_rate", rate);
    
    // Subscribers
    ros::Subscriber vel_sub = n.subscribe("two_link_vs/j_vel", 1, getVelocity);
    ros::Subscriber flag_sub = n.subscribe("two_link_vs/end_flag", 1, getFlag);

    // Publishers
    j1_vel_pub = n.advertise<std_msgs::Float64>("two_link/joint1_velocity_controller/command", 1);
    j2_vel_pub = n.advertise<std_msgs::Float64>("two_link/joint2_velocity_controller/command", 1);
    
    std::cout<<"sim interface ready"<<std::endl;

    ros::spin();
    return 0;

}