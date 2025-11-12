#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

#define MAX_PWM 255
#define FEATURES 2
#define VELOCITY_CONTROL 2


int main(int argc, char **argv){
    ros::init(argc, argv, "motion_test");
    ros::NodeHandle n;
    
    // Service server
    // std::cout<<"Estimation service initialized"<<std::endl;
    ros::Duration(25).sleep();

    // Publishers
    ros::Publisher vel_pub =  n.advertise<std_msgs::UInt8MultiArray>("origami_vs/velocity",1);
    int itr = 0;
    int window_size = 0;
    float rate = 0.0;
    float adaptive_gain = 0.0;
    std::vector<double> qhat;
    
    // Read parameters
    n.getParam("origami_adaptive_vs/window_size", window_size);
    n.getParam("origami_skeleton_vs/control_rate", rate);
    n.getParam("origami_adaptive_vs/init_Jacobian", qhat);
    n.getParam("origami_adaptive_vs/gamma", adaptive_gain);

    float param = 0.3;
    float j1_vel = 0.0, j2_vel = 0.0;
    // std::cout<<"old pose: "<<cur_ee_pose[0]<<", "<<cur_ee_pose[1]<<std::endl;
    // Velocity message
    std_msgs::UInt8MultiArray velocity;

    ros::Rate r{rate};
    float t= 1/rate;
    // std::cout<<"window:"<<window_size<<std::endl;
    // Command velocities and record state variables to windows
    while(itr<window_size){

        // if(itr < window_size/2){
            j1_vel = MAX_PWM * sin(param);
            j2_vel = MAX_PWM * cos(param);
        // }
        // else{
            // j1_vel = MAX_PWM * cos(param);
            // j2_vel = MAX_PWM * sin(param);
        // }
        // std::cout<<  "vel 1: " << j1_vel << ", vel 2: " <<j2_vel <<std::endl;
        velocity.data.clear();
        
        // Actuator 1
        if(j1_vel>0)
            velocity.data.push_back(0);
        else
            velocity.data.push_back(1);
        
        velocity.data.push_back(uint(abs(j1_vel)));
        // std::cout<<"Pushing vel 1"<<std::endl;
        // Actuator 2
        if(j1_vel + j2_vel>0)
            velocity.data.push_back(0);
        else
            velocity.data.push_back(1);

        velocity.data.push_back(uint(abs((j1_vel+j2_vel)/2)));
        
        // Actuator 3
        if(j2_vel>0)
            velocity.data.push_back(0);
        else
            velocity.data.push_back(1);

        velocity.data.push_back(uint(abs(j2_vel))); //new setup
        
        // Control mode
        velocity.data.push_back(VELOCITY_CONTROL);
        vel_pub.publish(velocity);
        
        param = param + 0.2;
        itr = itr+1;
        ros::spinOnce();
        r.sleep();
    }

    for(int i=0; i<6;i++){
        velocity.data.push_back(0);
    }
    velocity.data.push_back(VELOCITY_CONTROL);
    vel_pub.publish(velocity);


    ros::spin();
    return 0;
}