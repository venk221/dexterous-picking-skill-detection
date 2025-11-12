#include "ros/ros.h"
// #include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

#define MAX_PWM 2047

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_test");
    ros::NodeHandle n;
    
    // Publishers
    ros::Publisher vel_pub =  n.advertise<std_msgs::Int32MultiArray>("origami_vs/OMMD_velocity",1);
    int itr = 0;
    int rate = 10;
    int window_size=100;
    // Read parameters
    // n.getParam("origami_adaptive_vs/window_size", window_size);
    // n.getParam("origami_skeleton_vs/control_rate", rate);

    float param = 0.3;
    float v1 = 0.0, v2 = 0.0;
    
    std_msgs::Int32MultiArray velocity;

    ros::Rate r{rate};
    float t= 1/rate;

    while(itr<window_size){
        
        v1 = MAX_PWM*sin(param);
        v2 = MAX_PWM*cos(param);

        velocity.data.clear();
        // Module 0
        velocity.data.push_back(0);
        for(int i=0;i<3;i++){
            velocity.data.push_back(0);
        }
        // Module 1
        velocity.data.push_back(0);
        for(int i=0;i<3;i++){
            velocity.data.push_back(0);
        }
        // Module 2
        velocity.data.push_back(0);
        for(int i=0;i<3;i++){
            velocity.data.push_back(int(v1));
        }
        // Module 3
        velocity.data.push_back(0);
        for(int i=0;i<3;i++){
            velocity.data.push_back(int(v2));
        }

        // padding
        for(int i=0; i<16;i++){
            velocity.data.push_back(0);
        }

        vel_pub.publish(velocity);

        param = param + 0.2;
        itr = itr+1;
        ros::spinOnce();
        r.sleep();
    }

    velocity.data.clear();
    for(int i=0; i<32; i++){
        velocity.data.push_back(0);
    }
    vel_pub.publish(velocity);

    // ros::spinOnce();

    


    ros::spin();
    return 0;
}