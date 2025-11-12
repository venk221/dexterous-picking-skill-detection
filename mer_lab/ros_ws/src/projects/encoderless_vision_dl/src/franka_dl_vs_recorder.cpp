#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>

#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

int status = 0;


void statusCallback(const std_msgs::Int32 &status_msg){
    status = status_msg.data;
}


void dsCallback(const std_msgs::Float64MultiArray &msg){
    // Write ds to excel
    std::ofstream ds_plotdata("ds.csv",std::ios::app);
    // uncomment next line for 3 features
    ds_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<","<<msg.data.at(2)<<","<<msg.data.at(3)<<","<<msg.data.at(4)<<","<<msg.data.at(5)<<"\n";

    // uncommnet next line for 4 features
    // ds_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<","<<msg.data.at(2)<<","<<msg.data.at(3)<<","
    // <<msg.data.at(4)<<","<<msg.data.at(5)<<msg.data.at(6)<<","<<msg.data.at(7)<<","
    // <<msg.data.at(8)<<","<<msg.data.at(9)<<"\n";
    // ds_plotdata.close();
}
void drCallback(const std_msgs::Float64MultiArray &msg){
    // Write dr to excel
    std::ofstream dr_plotdata("dr.csv", std::ios::app);
    dr_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<"\n"; //uncomment for 2 joints
    // dr_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<","<<msg.data.at(2)<<"\n";//uncomment for 3 joints    
    dr_plotdata.close();
}
void JCallback(const std_msgs::Float32 &msg){
    if(status > 0){
        std::ofstream J_plotdata("modelerror.csv",std::ios::app);
        J_plotdata<<msg.data<<"\n";
        J_plotdata.close();
    }
}

// void j1velCallback(const std_msgs::Float64 &msg){
//     std::ofstream j1vel_plotdata("j1vel.csv",std::ios::app);
//     j1vel_plotdata<<msg.data<<"\n";
//     j1vel_plotdata.close();
// }

// void j2velCallback(const std_msgs::Float64 &msg){
//     std::ofstream j2vel_plotdata("j2vel.csv",std::ios::app);
//     j2vel_plotdata<<msg.data<<"\n";
//     j2vel_plotdata.close();
// }

// Uncomment - this block is to save data for 2 joints
void velCallback(const std_msgs::Float64MultiArray &msg){
    if(status>0){
        std::ofstream j1vel_plotdata("j1vel.csv",std::ios::app);
        j1vel_plotdata<<msg.data.at(0)<<"\n";
        j1vel_plotdata.close();

        std::ofstream j2vel_plotdata("j2vel.csv",std::ios::app);
        j2vel_plotdata<<msg.data.at(1)<<"\n";
        j2vel_plotdata.close();
    }
}

// Unomment - this block is to save data for 3 joints
// void velCallback(const std_msgs::Float64MultiArray &msg){
//     if(status>0){
//         std::ofstream j1vel_plotdata("j1vel.csv",std::ios::app);
//         j1vel_plotdata<<msg.data.at(0)<<"\n";
//         j1vel_plotdata.close();

//         std::ofstream j2vel_plotdata("j2vel.csv",std::ios::app);
//         j2vel_plotdata<<msg.data.at(1)<<"\n";
//         j2vel_plotdata.close();
        
//         std::ofstream j3vel_plotdata("j3vel.csv",std::ios::app);
//         j3vel_plotdata<<msg.data.at(1)<<"\n";
//         j3vel_plotdata.close();


//     }
// }
//  Uncomment the next block for 3f 2j
void errCallback(const std_msgs::Float64MultiArray &msg){
    if(status>1){
        std::ofstream err_plotdata("err.csv", std::ios::app);
        err_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<","
                    <<msg.data.at(2)<<","<<msg.data.at(3)<<","
                    <<msg.data.at(4)<<","<<msg.data.at(5)<<"\n";
        err_plotdata.close();
    }
}

// Uncomment the next block for 4f 3j
// void errCallback(const std_msgs::Float64MultiArray &msg){
//     if(status>1){
//         std::ofstream err_plotdata("err.csv", std::ios::app);
//         err_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<","
//                     <<msg.data.at(2)<<","<<msg.data.at(3)<<","
//                     <<msg.data.at(4)<<","<<msg.data.at(5)<<","
//                     <<msg.data.at(6)<<","<<msg.data.at(7)<<","
//                     <<msg.data.at(8)<<","<<msg.data.at(9)<<"\n";
//         err_plotdata.close();
//     }
// }

//  Uncomment the next block for 3f 2j
void cpCallback(const std_msgs::Float64MultiArray &msg){
        if(status>0){
            std::ofstream cp_plotdata("cp.csv", std::ios::app);
            cp_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<","<<msg.data.at(2)<<","
            <<msg.data.at(3)<<","<<msg.data.at(4)<<","<<msg.data.at(5)<<"\n";
            cp_plotdata.close();
        }
}

//Uncomment the next block for 4f 3j
// void cpCallback(const std_msgs::Float64MultiArray &msg){
//         if(status>0){
//             std::ofstream cp_plotdata("cp.csv", std::ios::app);
//             cp_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<","<<msg.data.at(2)<<","
//             <<msg.data.at(3)<<","<<msg.data.at(4)<<","<<msg.data.at(5)<<","<<msg.data.at(6)<<","
//             <<msg.data.at(7)<<","<<msg.data.at(8)<<","<<msg.data.at(9)<< "\n";
//             cp_plotdata.close();
//         }
// }

int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle n;

    // std::cout<<"Creating files"<<std::endl;
    // Create files to write data to
    std::ofstream J_plot("modelerror.csv");
    std::ofstream ds_plot("ds.csv");
    std::ofstream dr_plot("dr.csv");
    std::ofstream j1vel_plot("j1vel.csv");
    std::ofstream j2vel_plot("j2vel.csv");
    // std::ofstream j3vel_plot("j3vel.csv"); // comment/uncomment on the basis of joint numbers
    std::ofstream err_plot("err.csv");
    std::ofstream cp_plot("cp.csv");

    // Add column names to files
    J_plot <<"Model Error"<<"\n";
    J_plot.close();
    // ds_plot <<"ds_x"<<","<<"ds_y"<<"\n";

    // Uncomment the next block for 3f 2j
    ds_plot <<"cp2 x"<<","<<"cp2 y"<< ","<<"cp3 x"<<","<<"cp3 y"<<","<<"cp4 x"<<","<<"cp4 y"<<"\n";
    ds_plot.close();
    dr_plot <<"dr_1"<<","<<"dr_2"<<"\n";
    dr_plot.close();
    j1vel_plot <<"Joint 1" <<"\n";
    j1vel_plot.close();
    j2vel_plot <<"Joint 2" <<"\n";
    j2vel_plot.close();
    // j3vel_plot <<"Joint 3" <<"\n";
    // j3vel_plot.close();
    err_plot <<"Err_cp2_x"<<","<<"Err_cp2_y"<<","<<"Err_cp3_x"<<","<<"Err_cp3_y"
             << ","<<"Err_cp4_x"<<","<<"Err_cp4_y"<<"\n";
    err_plot.close();
    cp_plot <<"cp2 x"<<","<<"cp2 y"<< ","<<"cp3 x"<<","<<"cp3 y"<<","<<"cp4 x"<<","<<"cp4 y"<<"\n";
    cp_plot.close();

    // Uncomment next block for 4f 3j
    // ds_plot <<"cp2 x"<<","<<"cp2 y"<<","<<"cp3 x"<<","<<"cp3 y"<<","<<"cp4 x"<<","<<"cp4 y"<<","
    //             <<"cp5 x"<<","<<"cp5 y"<<","<<"cp6 x"<<","<<"cp6 y"<<"\n";
    // ds_plot.close();
    // dr_plot <<"dr_1"<<","<<"dr_2"<<","<<"dr_3"<<"\n";
    // dr_plot.close();
    // j1vel_plot <<"Joint 1" <<"\n";
    // j1vel_plot.close();
    // j2vel_plot <<"Joint 2" <<"\n";
    // j2vel_plot.close();
    // j3vel_plot <<"Joint 3" <<"\n";
    // j3vel_plot.close();
    // err_plot <<"Err_cp2_x"<<","<<"Err_cp2_y"<<","<<"Err_cp3_x"<<","<<"Err_cp3_y"
    //          <<","<<"Err_cp4_x"<<","<<"Err_cp4_y"<<","<<"Err_cp5_x"<<","<<"Err_cp5_y"<<","
    //          <<"\n";
    // err_plot.close();
    // cp_plot <<"cp2 x"<<","<<"cp2 y"<<","<<"cp3 x"<<","<<"cp3 y"<<","<<"cp4 x"<<","<<"cp4 y"<<","
    //             <<"cp5 x"<<","<<"cp5 y"<<","<<"cp6 x"<<","<<"cp6 y"<<"\n";
    // cp_plot.close();



    // Initialize subscribers
    ros::Subscriber ds_sub = n.subscribe("ds_record",1,dsCallback);
    ros::Subscriber dr_sub = n.subscribe("dr_record",1,drCallback);
    ros::Subscriber J_sub = n.subscribe("J_modelerror",1,JCallback);
    ros::Subscriber j_vel_sub = n.subscribe("joint_vel", 1, velCallback);
    ros::Subscriber error_sub = n.subscribe("servoing_error",1,errCallback);
    ros::Subscriber status_sub = n.subscribe("vsbot/status", 1, statusCallback);
    ros::Subscriber cp_sub = n.subscribe("vsbot/control_points", 1, cpCallback);
                                    // This is for storing feature trajectories 

    // Also add subs for Jacobian and Jacobian update

    ros::spin();
    return 0;
}