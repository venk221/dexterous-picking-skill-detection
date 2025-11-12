#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

bool target_is_stationary = true;
std::vector<float> target;
int rate = 0;
int no_of_features = 0;
bool end_flag = false;
bool start_flag = false;
float th = 0.0;
float target_step = 0.0;
float range = 0.0;
float radius = 0.0;

void getEnd(const std_msgs::Bool &msg){
    end_flag = msg.data;
}

void getStart(const std_msgs::Bool &msg){
    start_flag = msg.data;
    std::cout<<msg.data<<std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ref_trajectory");
    ros::NodeHandle n;

    n.getParam("two_link_vs/stationary_target", target_is_stationary);
    n.getParam("two_link_vs/target_init", target);
    n.getParam("two_link_vs/control_rate", rate);
    n.getParam("two_link_vs/no_of_features", no_of_features);
    n.getParam("two_link_vs/slope", th);
    n.getParam("two_link_vs/target_speed", target_step);
    n.getParam("two_link_vs/target_range", range);
    n.getParam("two_link_vs/radius", radius);

    ros::Subscriber flag_sub = n.subscribe("two_link_vs/end_flag", 1, getEnd);
    ros::Subscriber start_sub = n.subscribe("two_link_vs/start_flag", 1, getStart);
    
    ros::Publisher target_pub = n.advertise<std_msgs::Float64MultiArray>("two_link_vs/reference", 1);
    
    ros::Rate r{rate};

    std_msgs::Float64MultiArray target_msg;
    float s = target_step; // parameter for choosing point on line
    while(!start_flag){
        r.sleep();
        ros::spinOnce();
    }
    while(start_flag){
        
        if(target_is_stationary){
            target_msg.data.clear();
            for(int i=0; i<no_of_features; i++){
                target_msg.data.push_back(target[i]);
            }
            target_pub.publish(target_msg);
        }
        else{
            if(s<range){
                target_msg.data.clear();
                // function for target trajectory
                // std::cout<<(target[0] + s*cos(th*3.14/180))<<std::endl;
                
                // For a circular trajectory
                // s ranges from 0 - 2pi
                // target is center of circle (robots base location)
                target_msg.data.push_back(target[0] + radius*cos(s));
                target_msg.data.push_back(target[1] + radius*sin(s));

                // For straight line traj
                // s is the lenth of line segment
                // target_msg.data.push_back(target[0] + s*cos(th*3.14/180));
                // target_msg.data.push_back(target[1] + s*sin(th*3.14/180));
                target_pub.publish(target_msg);

                s = s + target_step;
            }
        }

        r.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}