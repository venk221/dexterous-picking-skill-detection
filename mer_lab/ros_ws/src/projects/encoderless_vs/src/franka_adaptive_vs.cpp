#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>
#include <vector>
#include "encoderless_vs/energyFuncMsg.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "encoderless_vs/vel_start.h"

// Status List - 
//  0 - Experiment not started
//  1 - Initial estimation period
//  2 - Visual servoing period
// -1 - Visual servoing completed

float eemarkerX;        // End effector x-pixel co-ordinate
float eemarkerY;        // End effector y-pixel co-ordinate
bool end_flag = false;  // true when exp is completeds

void endflagCallback(const std_msgs::Bool &msg){
    end_flag = msg.data;
}

void eeMarkerCallback(const std_msgs::Float32MultiArray &msg){
    eemarkerX = msg.data.at(0);
    eemarkerY = msg.data.at(1);
}

int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle n;
    
    // waiting for services and Gazebo
    ros::Duration(5).sleep(); 

    // Initializing ROS publishers
    ros::Publisher j_pub = n.advertise<std_msgs::Float64MultiArray>("joint_vel", 1);
    ros::Publisher ds_pub = n.advertise<std_msgs::Float64MultiArray>("ds_record", 1);
    ros::Publisher dr_pub = n.advertise<std_msgs::Float64MultiArray>("dr_record", 1);
    ros::Publisher J_pub = n.advertise<std_msgs::Float32>("J_modelerror",1);
    ros::Publisher err_pub = n.advertise<std_msgs::Float64MultiArray>("servoing_error", 1);
    ros::Publisher status_pub = n.advertise<std_msgs::Int32>("vsbot/status", 1);
    // std::cout << "Initialized Publishers" <<std::endl;

    // Initializing ROS subscribers
    ros::Subscriber ee_marker = n.subscribe("aruco/Pose", 1, eeMarkerCallback);
    ros::Subscriber end_sub = n.subscribe("vsbot/end_flag", 1, endflagCallback);

    // std::cout << "Initialized Subscribers" <<std::endl;
    
    // Initializing service clients
    ros::service::waitForService("computeEnergyFunc",1000);
    ros::ServiceClient energyClient = n.serviceClient<encoderless_vs::energyFuncMsg>("computeEnergyFunc");
    // std::cout << "Initialized Service Clients"<< std::endl;

    // Initializing status msg
    std_msgs::Int32 status;
    status.data = 0;
    status_pub.publish(status);

    // Servoing variables 
    float gamma; // learning rate for initial estimation
    n.getParam("vsbot/estimation/gamma", gamma);
    float gamma2; // learning rate during visual servoing
    n.getParam("vsbot/estimation/gamma2", gamma2);
    int window; // Estimation window size
    n.getParam("vsbot/estimation/window", window);
    int it = 0; // iterator
    std::vector<float> error (2,0); //error vector
    float err = 0.0; // error norm
    float rate; // control loop rate
    n.getParam("vsbot/estimation/rate",rate);
    float thresh;
    n.getParam("vsbot/control/thresh",thresh);
    std::vector<float> goal (2,0);
    n.getParam("vsbot/control/goal_pos_x", goal[0]);
    n.getParam("vsbot/control/goal_pos_y", goal[1]);
    float lam; // servoing gain
    n.getParam("vsbot/control/lam",lam);
    float saturation; // Error saturation value
    n.getParam("vsbot/control/saturation", saturation);

    std_msgs::Float64MultiArray err_msg;
    // std::cout << "Initialized Servoing Variables" << std::endl;

    // Initial estimation variables
    std::vector<float> ds; // change in ee pose
    std::vector<float> dr; // change in joint angles
    std::vector<float> qhat {7,13,-21,17}; // initial Jacobian matrix as vector
    // std::vector<float> qhat {0.7,1.3,-2.1,1.7};
    std::vector<float> dSinitial; // Vector list of shape change vectors
    std::vector<float> dRinitial; // Vector list of velocity change vectors
    std_msgs::Float64MultiArray j_vel;
    std_msgs::Float64MultiArray ds_msg;
    std_msgs::Float64MultiArray dr_msg;
    float old_angle_j1 = 0.0;
    float old_angle_j2 = 0.0;
    float markerX_cur;
    float markerY_cur;
    float t = 1/rate; // time in seconds, used for integrating angular velocity
    
    // Jani update putting the velocity amplitude in the config.yaml file
    float amplitude;
    n.getParam("vsbot/estimation/amplitude", amplitude);

    std::cout <<"Initialized estimation variable" << std::endl;
    
    std::cout <<"Sleeping for 5 seconds" << std::endl;
    ros::Duration(5).sleep();  // Allow the camera to initialize

// ----------------------- Setting Joint Stiffness --------------------------
    ros::service::waitForService("velocity_start_service",1000);
    ros::ServiceClient set_stiffness = n.serviceClient<encoderless_vs::vel_start>("velocity_start_service");

    encoderless_vs::vel_start stiffness_msg;
    stiffness_msg.request.input = true;
    set_stiffness.call(stiffness_msg);
    // Stiffness is set continue with experiment procedure
    
// --------------------------- Initial Estimation -----------------------------
    // Refresh subscribers
    ros::spinOnce();
    
    // Obtain initial robot state
    float markerX_old = eemarkerX;
    float markerY_old = eemarkerY;
    
    // command small displacements around initial position
    ros::Rate r{rate};  // Rate for control loop
    std::cout << "Ready to command small displacements" <<std::endl;

    // Jani - applied joint velocity limit
    float max_vel_lim = amplitude;
    
    status.data = 1;
    float param = 0.3;

    while(it < window)
    {   
        // Publish sin vel w/ random noise to both joints
        float j1_vel = amplitude*sin(param);
        float j2_vel = amplitude*cos(param);

        j_vel.data.clear();
        j_vel.data.push_back(j1_vel);
        j_vel.data.push_back(j2_vel);

        param = param + 0.1;

        j_pub.publish(j_vel);        

        // For adding noise to the generated velocity
        // Keep in mind the SNR while adding noise

        // j1_vel.data = (2.0*((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 0.5));
        // j2_vel.data = (2.0*((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 0.5));
        // j1_pub.publish(j1_vel);
        // j2_pub.publish(j2_vel);
        
        // Obtain current robot state
        markerX_cur = eemarkerX;
        markerY_cur = eemarkerY;

        // Compute change in state
        // End-effector position
        ds.clear();
        ds.push_back((markerX_cur - markerX_old));
        ds.push_back((markerY_cur - markerY_old));

        // Joint angle
        dr.clear();
        dr.push_back((j1_vel*t));
        dr.push_back((j2_vel*t));

        // Update dSinitial and dRinitial
        dSinitial.push_back(ds[0]);
        dSinitial.push_back(ds[1]);
        dRinitial.push_back(dr[0]);
        dRinitial.push_back(dr[1]);

        // Update state variables
        markerX_old = markerX_cur;
        markerY_old = markerY_cur;
        
        // Publish ds, dr vectors to store
            // Convert to Float64multiarray
            ds_msg.data.clear();
            ds_msg.data.push_back(ds[0]);
            ds_msg.data.push_back(ds[1]);
            dr_msg.data.clear();
            dr_msg.data.push_back(dr[0]);
            dr_msg.data.push_back(dr[1]);
            // publish
            ds_pub.publish(ds_msg);
            dr_pub.publish(dr_msg);
            
        // publish status msg
        status_pub.publish(status);
        
        // Increase iterator
        it++;

        // Refresh subscribers
        ros::spinOnce();
        r.sleep();
    }

    // Commanding 0 velocity to robot 

    j_vel.data.clear();
    j_vel.data.push_back(0.0);
    j_vel.data.push_back(0.0);

    j_pub.publish(j_vel);
    
    std::cout<<"Initial Movements Complete"<<std::endl;

    // Declare ROS Msg Arrays
    std_msgs::Float32MultiArray dSmsg;
    dSmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dSmsg.layout.dim[0].label = "dS_elements";
    dSmsg.layout.dim[0].size = dSinitial.size();
    dSmsg.layout.dim[0].stride = 1;
    dSmsg.data.clear();
    
    std_msgs::Float32MultiArray dRmsg;
    dRmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dRmsg.layout.dim[0].label = "dR_elements";
    dRmsg.layout.dim[0].size = dRinitial.size();
    dRmsg.layout.dim[0].stride = 1;
    dRmsg.data.clear();

    std_msgs::Float32MultiArray qhatmsg;
    qhatmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    qhatmsg.layout.dim[0].label = "qhat_elements";
    qhatmsg.layout.dim[0].size = qhat.size();
    qhatmsg.layout.dim[0].stride = 1;
    qhatmsg.data.clear();

    // std::cout << "Declared ROS msg arrays" <<std::endl;

    // Push data to ROS Msg
    for(std::vector<float>::iterator itr = dSinitial.begin(); itr != dSinitial.end(); ++itr){
        // std::cout <<*itr<<std::endl;
        dSmsg.data.push_back(*itr);
    }
    
    for(std::vector<float>::iterator itr = dRinitial.begin(); itr != dRinitial.end(); ++itr){
        // std::cout <<*itr<<std::endl;
        dRmsg.data.push_back(*itr);
    }

    for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
        // std::cout <<*itr<<std::endl;
        qhatmsg.data.push_back(*itr);
    }
    std::cout <<"Pushed initial data to ROS msgs"<<std::endl;

    // Compute Jacobian
    it = 0;
    encoderless_vs::energyFuncMsg msg;
    while(it < window){
        // Service request data
        msg.request.gamma = gamma;
        msg.request.it = it;
        msg.request.dS = dSmsg;
        msg.request.dR = dRmsg;
        msg.request.qhat = qhatmsg;

        // call compute energy functional
        energyClient.call(msg);

        // Populating service response
        std::vector<float> qhatdot = msg.response.qhat_dot.data;

        //  Jacobian update
        for(int i = 0; i<qhat.size(); i++){
            qhat[i] = qhat[i] + qhatdot[i]; // Updating each element of Jacobian
        }
        // std::cout<<"Updated Jacobian vector:";

        // Push updated Jacobian vector to ROS Msg
        qhatmsg.data.clear();
        for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
            // std::cout <<*itr<<",";
            qhatmsg.data.push_back(*itr);
        }
        // std::cout<< std::endl;

        // Publish J value to store
        std_msgs::Float32 J;
        J.data = msg.response.J;
        J_pub.publish(J);
        
        it++;
    }
    std::cout <<"Initial Estimation Completed" << std::endl;

// ----------------------------- Start Servoing ---------------------------------- 
    // Compute error and error norm at start of servoing
    // Refresh subscribers
    ros::spinOnce();

    // Initial feature error
    error[0] = eemarkerX - goal[0];
    error[1] = eemarkerY - goal[1];
    std::complex<float> temp (error[0],error[1]);
    err = sqrt(std::norm(temp));
    
    std::cout<<"Entering control loop"<<std::endl;
    
    status.data = 2;
    status_pub.publish(status);
    
    while(!end_flag){    // convergence condition

        markerX_cur = eemarkerX;
        markerY_cur = eemarkerY;

        // error norm "err" is always positive
        // compute current error & norm
        error[0] = markerX_cur - goal[0];
        error[1] = markerY_cur - goal[1];

        std::complex<float> err_temp (error[0],error[1]);
        err = sqrt(std::norm(err_temp));
        
        // Publish current error
        err_msg.data.clear();
        err_msg.data.push_back(error[0]);
        err_msg.data.push_back(error[1]);
        err_pub.publish(err_msg);

        // Generate velocity
        // Convert qhat vector into matrix format
        Eigen::MatrixXf Qhat(2,2);

        int row_count = 0;
        int itr = 0;
        while(row_count<2){
            Qhat.row(row_count) << qhat[itr], qhat[itr+1];
            row_count = row_count + 1;
            itr = itr + 2;
        }
        Eigen::Vector2f error_vec(error[0], error[1]);
        Eigen::Vector2f joint_vel;
        // Abhinav implementing error sturation - 2/22/2022
        // joint_vel = lam*(Qhat.inverse())*(error_vec);        
        // Eigen::Vector2f cart_vel;
        // cart_vel = error_vec;

        if(abs(error_vec[0]) > saturation){
            error_vec[0] = (error_vec[0]/abs(error_vec[0]))*saturation;
        }
        if(abs(error_vec[1] > 1.0)){
            error_vec[1] = (error_vec[1]/abs(error_vec[1]))*saturation;
        }
        joint_vel = lam*Qhat.inverse()*error_vec;
    //     // Jani - applying sliding mode controller
    //     // if (joint_vel[0] > 0) {            
    //     //     joint_vel[0] = max_vel_lim;
    //     // } 
    //     // else if (joint_vel[0] < 0) {
    //     //    joint_vel[0] = -max_vel_lim;
    //     // }

    //     // if (joint_vel[1] > 0) {            
    //     //     joint_vel[1] = max_vel_lim;
    //     // } 
    //     // else if (joint_vel[1] < 0) {
    //     //    joint_vel[1] = -max_vel_lim;
    //     // }
/*
    //     // Jani applying normalized velocity
        float vel_sum = abs(joint_vel[0]) + abs(joint_vel[1]);
        if(vel_sum > 0){
            joint_vel[0] = joint_vel[0]/vel_sum;
            // std::cout<<"normalized joint1 vel:"<<joint_vel[0]<<"\n";
            joint_vel[0] = joint_vel[0]*(max_vel_lim);
            // std::cout<<"capped normalized joint1 vel:"<<joint_vel[0]<<"\n";
            joint_vel[1] = joint_vel[1]/vel_sum;
            // std::cout<<"normalized joint2 vel:"<<joint_vel[1]<<"\n";
            joint_vel[1] = joint_vel[1]*(max_vel_lim);
            // std::cout<<"capped normalized joint2 vel:"<<joint_vel[1]<<"\n";
        }
*/


  /*      // Abhinav implementing a saturated P-controller 2-21-2022
        // This controller acts as SM controller for large errors
        // and converts to a P-controller closer to the ref
        // In the if blocks, first term determines the sign of the vel

        if(abs(joint_vel[0]) > max_vel_lim){
            joint_vel[0] = (joint_vel[0]/abs(joint_vel[0])) * max_vel_lim;
        }
        if(abs(joint_vel[1]) > max_vel_lim){
            joint_vel[1] = (joint_vel[1]/abs(joint_vel[1])) * max_vel_lim;
        }
*/
    //     // Publish velocity to robot
        j_vel.data.clear();
        j_vel.data.push_back(joint_vel[0]);
        j_vel.data.push_back(joint_vel[1]);
        j_pub.publish(j_vel);
        
        // Compute change in state
        ds.clear();
        ds.push_back((markerX_cur - markerX_old));
        ds.push_back((markerY_cur - markerY_old));
        
        dr.clear();
        dr.push_back((joint_vel[0]*t));
        dr.push_back((joint_vel[1]*t));
        // Publish ds, dr vectors to store
            // Convert to Float64multiarray
            ds_msg.data.clear();
            ds_msg.data.push_back(ds[0]);
            ds_msg.data.push_back(ds[1]);
            dr_msg.data.clear();
            dr_msg.data.push_back(dr[0]);
            dr_msg.data.push_back(dr[1]);
            // publish
            ds_pub.publish(ds_msg);
            dr_pub.publish(dr_msg);

        if(err > thresh){
            // Update dSinitial and dRinitial
            dSinitial[0] = ds[0];
            dSinitial[1] = ds[1];
    
            std::rotate(dSinitial.begin(), dSinitial.begin()+2, dSinitial.end());
            
            dRinitial[0] = dr[0];
            dRinitial[1] = dr[1];
            
            std::rotate(dRinitial.begin(), dRinitial.begin()+2, dRinitial.end());
            
            markerX_old = markerX_cur;
            markerY_old = markerY_cur;

            // Compute Jacobian update
            // converting vectors to ros msg for service
            dSmsg.data.clear();
            for(std::vector<float>::iterator itr = dSinitial.begin(); itr != dSinitial.end(); ++itr){
                // std::cout <<*itr<<std::endl;
                dSmsg.data.push_back(*itr);
            }
            
            dRmsg.data.clear();
            for(std::vector<float>::iterator itr = dRinitial.begin(); itr != dRinitial.end(); ++itr){
                // std::cout <<*itr<<std::endl;
                dRmsg.data.push_back(*itr);
            }

            qhatmsg.data.clear();
            for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
                // std::cout <<*itr<<std::endl;
                qhatmsg.data.push_back(*itr);
            }

            // populating request data
            msg.request.gamma = gamma2;
            msg.request.it = window-1;
            msg.request.dS = dSmsg;
            msg.request.dR = dRmsg;
            msg.request.qhat = qhatmsg;

            // Call energy functional service
            energyClient.call(msg);
            
            // Populate service response
            std::vector<float> qhatdot = msg.response.qhat_dot.data;

            // Update Jacobian
            for(int i = 0; i<qhat.size(); i++){
                qhat[i] = qhat[i] + qhatdot[i]; // Updating each element of Jacobian
            }

            // Push updated Jacobian vector to ROS Msg
            qhatmsg.data.clear();
            for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
                // std::cout <<*itr<<",";
                qhatmsg.data.push_back(*itr);
            }
            // std::cout<< std::endl;
        }

        // Publish J
        std_msgs::Float32 J;
        J.data = msg.response.J;
        J_pub.publish(J);
        
        // Publish status msg
        status_pub.publish(status);

        // Refresh control loop variables, sleep to maintain control rate
        ros::spinOnce();
        r.sleep();
    }
    
    // Commanding 0 velocity to robot 
    j_vel.data.clear();
    j_vel.data.push_back(0.0);
    j_vel.data.push_back(0.0);

    j_pub.publish(j_vel);

    std::cout<<"Servoing Complete"<<std::endl;
    status.data = -1;
    status_pub.publish(status);

    // Shutdown

}