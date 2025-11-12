#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

std::vector<float> init_target;
std::vector<float> cur_target;
float rate = 0.0;
float radius = 0.0;
float target_step = 0.0;
bool stationary_target = true;
bool servo = false;

void flagCallback(const std_msgs::Bool &msg){
    servo = msg.data;
}

int main(int argc, char **argv){
    
    // Init ROS node
    ros::init(argc, argv, "sim_image_servo");
    ros::NodeHandle n;

    // Read params
    n.getParam("origami_sim/target_feature", init_target);
    n.getParam("origami_sim/control_rate", rate);
    n.getParam("origami_sim/stationary_target", stationary_target);
    n.getParam("origami_sim/radius", radius);
    n.getParam("origami_sim/target_speed", target_step);

    // Subscirbers
    ros::Subscriber flag_sub = n.subscribe("origami_vs/start_servo", 1, flagCallback);

    // Publishers
    ros::Publisher target_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/reference", 1);
    
    // Rate loop
    std_msgs::Float64MultiArray cur_target_msg;
    ros::Rate r{rate};

    float s = target_step;
    cur_target = init_target;
    
    while(!servo){
        r.sleep();
        ros::spinOnce();
    }

    while(ros::ok()){

        // make changes to target as reqd here
        if(!stationary_target){
            cur_target[0] = init_target[0] + radius*cos(s);
            cur_target[1] = init_target[1] + radius*sin(s);
            cur_target[2] = 0;
            // std::cout<<"moving target"<<std::endl;

            s = s + target_step;
        }

        cur_target_msg.data.clear();
        for(int i=0; i<cur_target.size(); i++){
            cur_target_msg.data.push_back(cur_target[i]);
        }

        target_pub.publish(cur_target_msg);

        r.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}