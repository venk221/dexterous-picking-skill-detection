#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>
#include <unordered_map>
#include "MPC.h"



float ee_pos_x;  // Feature cordinates in Pixel frame
float ee_pos_y;  // Feature cordinates in Image frame
float theta1;    // Joint -1 angle
float theta2;    // Joint -2 angle
std_msgs::Bool end_flag; // Terminate Control loop

/***Callback for End Effector Marker Point Topic***/
void ee_feature_callback(const std_msgs::Float64MultiArray &msg){
    ee_pos_x = msg.data.at(0);
    ee_pos_y = msg.data.at(1);
}

/***Callback for Joint angles Topic***/
void joint_states_callback(const sensor_msgs::JointState msg){
     theta1 = msg.position[0];
     theta2 = msg.position[1];
}

void control_loop_callback(const std_msgs::Bool msg){
     end_flag = msg;
}

int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle n;
    
    ros::Rate rateController = ros::Rate(10); // 10hz -100ms one loop 

    // Initializing ROS publishers
    ros::Publisher j1_pub = n.advertise<std_msgs::Float64>("/planarbot/joint1_velocity_controller/command",1);
    ros::Publisher j2_pub = n.advertise<std_msgs::Float64>("/planarbot/joint2_velocity_controller/command",1);
    ros::Publisher err_pub = n.advertise<std_msgs::Float64MultiArray>("servoing_error", 1);
    ros::Publisher end_flag_pub = n.advertise<std_msgs::Bool>("control_loop_end_flag", 1);

    // Subscriber Node for End effector feature pose
    ros::Subscriber ee_feature = n.subscribe("ee_feature_pose",1,ee_feature_callback);
    ros::Subscriber joints_poses = n.subscribe("/planarbot/joint_states",1,joint_states_callback);
    ros::Subscriber control_loop = n.subscribe("/control_loop_end_flag",1,control_loop_callback);
    ros::Duration(3).sleep(); 
    //Error msg topic
    std_msgs::Float64MultiArray err_msg; 
    err_msg.data.clear();
    err_msg.data.push_back(0);
    err_msg.data.push_back(0);
    err_pub.publish(err_msg);
    // Declarations
    ros::WallTime start_,end_;
    std_msgs::Float64 j1_vel; // joint1 velocity
    std_msgs::Float64 j2_vel; // joint2 velocity
    
    Eigen::MatrixXf k_intrinsic(3,3);
    Eigen::Vector2f error_vec;
    Eigen::Vector2f pixel_pose;
    Eigen::Vector2f joint_vel;
    Eigen::MatrixXf J_robot(2,2); // Robot Jacobian
    Eigen::MatrixXf J_img(2,6); // Image Jacobian
    Eigen::MatrixXf J_combined(2,2); // Image Jacobian * Robot Jacobian

    float err;  // Norm of Pixel(x,y) Errors
    float ee_img_x; // Feature cordinates in Image frame
    float ee_img_y; // Feature cordinates in Image frame

    /** Gazebo Camera Parameters from Intrinsic Matrix **/
    unordered_map<string,double> mpc_param;
    n.getParam("planarbot/gazebo_camera/focus",mpc_param["focus"]); // Focus from Gazebo Camera info
    n.getParam("planarbot/gazebo_camera/c_intrinsic",mpc_param["c_intrinsic"]);//principle point centre offset
    n.getParam("planarbot/gazebo_camera/depth",mpc_param["depth"]);// depth-2
    //float focus_const = focus/Z; // 178.76/1.83

    /** Parameters -Robot **/
    n.getParam("planarbot/robot_params/l1",mpc_param["l1"]); // link-1 length
    n.getParam("planarbot/robot_params/l2",mpc_param["l2"]); // link-2 length

    /** Control loop Parameters **/
    n.getParam("planarbot/control/goal_pos_x",mpc_param["goal_pos_x"]); //Goal Feature positions  150,136
    n.getParam("planarbot/control/goal_pos_y",mpc_param["goal_pos_y"]);
    n.getParam("planarbot/control/thresh",mpc_param["thres"]); //Control loop Termination Threshold

    /** MPC  Parameters **/
    n.getParam("planarbot/mpc/mpc_steps",mpc_param["mpc_steps"]);
    n.getParam("planarbot/mpc/mpc_sample_time",mpc_param["mpc_dt"]);
    n.getParam("planarbot/mpc/w_pixel_x",mpc_param["w_pixel_x"]);
    n.getParam("planarbot/mpc/w_pixel_y",mpc_param["w_pixel_y"]);
    n.getParam("planarbot/mpc/w_j1_vel",mpc_param["w_j1_vel"]);
    n.getParam("planarbot/mpc/w_j2_vel",mpc_param["w_j2_vel"]);
    n.getParam("planarbot/mpc/w_j1_accl",mpc_param["w_j1_accl"]);
    n.getParam("planarbot/mpc/w_j2_accl",mpc_param["w_j2_accl"]);
    n.getParam("planarbot/mpc/j1_accl_limit",mpc_param["j1_accl_limit"]);
    n.getParam("planarbot/mpc/j2_accl_limit",mpc_param["j2_accl_limit"]);
    n.getParam("planarbot/mpc/pixel_x_lb",mpc_param["pixel_x_lb"]);
    n.getParam("planarbot/mpc/pixel_x_ub",mpc_param["pixel_x_ub"]);
    n.getParam("planarbot/mpc/pixel_y_lb",mpc_param["pixel_y_lb"]);
    n.getParam("planarbot/mpc/pixel_y_ub",mpc_param["pixel_y_ub"]);
    n.getParam("planarbot/mpc/theta_1_lb",mpc_param["theta_1_lb"]);
    n.getParam("planarbot/mpc/theta_1_ub",mpc_param["theta_1_ub"]);
    n.getParam("planarbot/mpc/theta_2_lb",mpc_param["theta_2_lb"]);
    n.getParam("planarbot/mpc/theta_2_ub",mpc_param["theta_2_ub"]);
    n.getParam("planarbot/mpc/j1_vel_lb",mpc_param["j1_vel_lb"]);
    n.getParam("planarbot/mpc/j1_vel_ub",mpc_param["j1_vel_ub"]);
    n.getParam("planarbot/mpc/j2_vel_lb",mpc_param["j2_vel_lb"]);
    n.getParam("planarbot/mpc/j2_vel_ub",mpc_param["j2_vel_ub"]);

    /** MPC **/
    Eigen::VectorXf state(6);
    vector<float> mpc_results{0,0,0,0};
    double execution_time; 
    /* Initial Error */
    error_vec[0] = mpc_param["goal_pos_x"] - ee_pos_x;
    error_vec[1] = mpc_param["goal_pos_y"] - ee_pos_y;
    err = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));

    /** Initialize Zero Joint Velcoities **/
    j1_vel.data = 0;
    j2_vel.data = 0;
    j1_pub.publish(j1_vel);
    j2_pub.publish(j2_vel);

    MPC _mpc(mpc_param);

    std::cout<<" Servoing Started "<<std::endl;
    /********** Control loop **************/
    while(int(end_flag.data) == 0){
   
        // update error 
        error_vec[0] = (mpc_param["goal_pos_x"] - ee_pos_x);
        error_vec[1] = (mpc_param["goal_pos_y"] - ee_pos_y);
        err = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));

        if((error_vec[0] != mpc_param["goal_pos_x"]) &&  (error_vec[1] != mpc_param["goal_pos_y"]))
        {
            // Publish current error to plot data
            err_msg.data.clear();
            err_msg.data.push_back(err);
            err_msg.data.push_back(error_vec[1]);
            err_pub.publish(err_msg);

            /**Robot Jacobian**/
            // J_robot << ((l1*cos(theta1))+(l2*cos(theta1+theta2))), ((l2*cos(theta1+theta2))),
            // ((l1*sin(theta1))+(l2*sin(theta1+theta2))) ,((l2*sin(theta1+theta2))),
            // State update
            // pixel_pose = (J_combined * joint_vel);
            state[0] = ee_pos_x;
            state[1] = ee_pos_y;
            state[2] = theta1;
            state[3] = theta2;
            state[4] = joint_vel[0];
            state[5] = joint_vel[1];

            start_ = ros::WallTime::now();
            // Solve MPC Problem
            mpc_results  = _mpc.Solve(state);
            joint_vel[0] = mpc_results[0];
            joint_vel[1] = mpc_results[1];
            end_ = ros::WallTime::now();
            execution_time = (end_ - start_).toNSec() * 1e-6;
            ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
            
            // Publish Joint Velocities
            j1_vel.data = joint_vel[0];
            j2_vel.data = joint_vel[1];
            j1_pub.publish(j1_vel);
            j2_pub.publish(j2_vel); 
        }
        ros::spinOnce();
        rateController.sleep();
    }

    j1_vel.data = 0.0;
    j2_vel.data = 0.0;
    j1_pub.publish(j1_vel);
    j2_pub.publish(j2_vel);
    std::cout<<" Servoing Completed "<<std::endl;
    ros::spin();
    return 0;
}
