#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

// This node publishes test sinusoidal velocities
// franka joint pub is used to cmd these velocities to the robot

int main(int argc, char **argv){

    ros::init(argc, argv, "franka_vel_test");
    ros::NodeHandle n;

    float param = 0.3;
    int it = 0;
    std_msgs::Float64MultiArray j_vel;

    ros::Publisher joint_vel_pub = n.advertise<std_msgs::Float64MultiArray>("joint_vel", 1);
    ros::Rate r{30};

    while(it< 90){

        float j1_vel = 0.5*sin(param + 1.7);
        float j2_vel = 0.5*cos(param - 0.5);
        
        j_vel.data.clear();
        j_vel.data.push_back(j1_vel);
        j_vel.data.push_back(j2_vel);

        joint_vel_pub.publish(j_vel);

        it += 1;
        param = param + 0.02;

        ros::spinOnce();
        r.sleep();
    }

    j_vel.data.clear();
    j_vel.data.push_back(0);
    j_vel.data.push_back(0);

    joint_vel_pub.publish(j_vel);

    std::cout<<"Complete"<<std::endl;
    
    ros::spin();
    return 0;
}