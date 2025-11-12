#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>

#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"


int status = 0;



void statusCallback(const std_msgs::Int32 &status_msg){
    status = status_msg.data;
    std::ofstream status_plotdata("status.csv",std::ios::app);
    status_plotdata<<status<<"\n";
    status_plotdata.close();
}

void dsCallback(const std_msgs::Float64MultiArray &msg){
    // Write ds to excel
    std::ofstream ds_plotdata("ds.csv",std::ios::app);
    ds_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<< "\n";
    ds_plotdata.close();
}
void drCallback(const std_msgs::Float64MultiArray &msg){
    // Write dr to excel
    std::ofstream dr_plotdata("dr.csv", std::ios::app);
    dr_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<"\n";
    dr_plotdata.close();
}

void errorCallback(const std_msgs::Float64MultiArray &error_msg){
    if (status > 1){
        std::ofstream err_plotdata("err.csv", std::ios::app);
        // float err_norm;
        // std::complex<float> temp (error_msg.data.at(0), error_msg.data.at(1));
        // err_norm = sqrt(std::norm(temp));
        err_plotdata <<error_msg.data.at(0)<<","<<error_msg.data.at(1)<<"\n"; //<<","<<err_norm<<"\n";
        err_plotdata.close();
    }
}


void modelErrCallback(const std_msgs::Float32 &J_msg){
    if (status > 0){
        std::ofstream J_plotdata("modelerror.csv", std::ios::app);
        J_plotdata <<J_msg.data<<"\n";
        J_plotdata.close();
    }
}


void poseCallback(const std_msgs::Float32MultiArray &pos_msg){
    if(status > 0){
        std::ofstream pos_plotdata("ee_pos.csv", std::ios::app);
        pos_plotdata <<pos_msg.data.at(0) <<"," <<pos_msg.data.at(1)<<"\n";
        pos_plotdata.close();
    }
}


void velCallback(const std_msgs::Float64MultiArray &jvel_msg){
    if(status > 0){
        std::ofstream j1vel_plotdata("j1vel.csv", std::ios::app);
        j1vel_plotdata <<jvel_msg.data.at(0) <<"\n";
        j1vel_plotdata.close();

        std::ofstream j2vel_plotdata("j2vel.csv", std::ios::app);
        j2vel_plotdata <<jvel_msg.data.at(1) <<"\n";
        j2vel_plotdata.close();
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "data_recorder");
    ros::NodeHandle n;

    // Create files to write data to
    std::ofstream J_plot("modelerror.csv");
    std::ofstream j1vel_plot("j1vel.csv");
    std::ofstream j2vel_plot("j2vel.csv");
    std::ofstream err_plot("err.csv");
    std::ofstream pos_plot("ee_pos.csv");
    std::ofstream ds_plot("ds.csv");
    std::ofstream dr_plot("dr.csv");
    std::ofstream status_plot("status.csv");

    // Add column names to files
    J_plot <<"Model Error"<<"\n";
    J_plot.close();

    j1vel_plot <<"Joint 1" <<"\n";
    j1vel_plot.close();
    
    j2vel_plot <<"Joint 2" <<"\n";
    j2vel_plot.close();

    err_plot <<"Err X," <<"Err Y," << "Err Norm" <<"\n";
    err_plot.close();

    pos_plot <<"X," << "Y" <<"\n";
    pos_plot.close();

    ds_plot <<"ds_x"<<","<<"ds_y"<<"\n";
    ds_plot.close();

    dr_plot <<"dr_x"<<","<<"dr_y"<<"\n";
    dr_plot.close();

    status_plot <<"Status"<<"\n";
    status_plot.close();

    // Declare subscribers
    ros::Subscriber error_sub = n.subscribe("servoing_error", 1, errorCallback);
    ros::Subscriber model_err_sub = n.subscribe("J_modelerror", 1, modelErrCallback);
    ros::Subscriber ee_sub = n.subscribe("aruco/Pose", 1, poseCallback);
    ros::Subscriber j_vel_sub = n.subscribe("joint_vel", 1, velCallback);
    ros::Subscriber status_sub = n.subscribe("vsbot/status", 1, statusCallback);
    ros::Subscriber ds_sub = n.subscribe("ds_record",1,dsCallback);
    ros::Subscriber dr_sub = n.subscribe("dr_record",1,drCallback);

    ros::spin();
    return 0;
}