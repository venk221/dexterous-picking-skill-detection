#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>
#include <vector>

#include "encoderless_vs/control_points.h"
#include "encoderless_vs/energyFuncMsg.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"


// Status List - 
//  0 - Experiment not started
//  1 - Initial estimation period
//  2 - Visual servoing period
// -1 - Visual servoing completed

// Declare global vector for spline features

int no_of_features; // = 4; // 3 control points in a plane, 
    // ignoring 1st control pt as it doesn't change much and can be discarded

bool start_flag = false; // true when camera is ready
bool end_flag = false;  // true when servoing is completed. Triggered by user


// float get_cur_th(float x1, float y1, float x2, float y2){
//     // function returns current angle of line wrt +ve x-axis
//     // line is drawn between first and last feature point
//     float base = abs(x2 - x1);
//     float hy = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
    
//     float th = acos(base/hy);

//     return th;
// }


int sign(double x){
    if(x<0)
        return -1;
    else if (x>0)
        return 1;
    else
        return 0;
}


void end_flag_callback(const std_msgs::Bool &msg){
    end_flag = msg.data;
}


void start_flag_callback(const std_msgs::Bool &msg){
    // This flag is true when camera is spawned and starts publishing
    start_flag = msg.data;
}


void print_fvector(std::vector<float> vec){
// function to print std::vector<float>
// this is commonly used for debugging
    for(std::vector<float>::iterator itr=vec.begin(); itr!=vec.end();++itr){
        std::cout<<*itr<<" "<<std::flush;
    }
}


int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "shape_servo_control_node");
    ros::NodeHandle n;

    // Initializing ROS publishers
    ros::Publisher j1_pub = n.advertise<std_msgs::Float64>("/vsbot/joint1_velocity_controller/command",1);
    ros::Publisher j2_pub = n.advertise<std_msgs::Float64>("/vsbot/joint2_velocity_controller/command",1);
    ros::Publisher ds_pub = n.advertise<std_msgs::Float64MultiArray>("ds_record", 1);
    ros::Publisher dr_pub = n.advertise<std_msgs::Float64MultiArray>("dr_record", 1);
    ros::Publisher J_pub = n.advertise<std_msgs::Float32>("J_modelerror",1);
    ros::Publisher err_pub = n.advertise<std_msgs::Float64MultiArray>("servoing_error", 1);
    ros::Publisher status_pub = n.advertise<std_msgs::Int32>("vsbot/status", 1);
    ros::Publisher cp_pub = n.advertise<std_msgs::Float64MultiArray>("vsbot/control_points", 1);
    // ros::Publisher dth_pub = n.advertise<std_msgs::Float32>("th_change_pub", 1);
    // std::cout << "Initialized Publishers" <<std::endl;

    // Initializing ROS subscribers
    // ros::Subscriber spline_features = n.subscribe("control_points",1,splineFeatureCallback);
    ros::Subscriber start_flag_sub = n.subscribe("vsbot/control_flag",1,start_flag_callback);
    ros::Subscriber end_flag_sub = n.subscribe("vsbot/end_flag",1,end_flag_callback);

    // waiting for camera to start publishing
    while(!start_flag){
        std::cout<<"waiting for camera to spawn"<<std::endl;
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    std::cout<<"Camera ready"<<std::endl;
    
    // waiting for services and Gazebo
    std::cout<<"Sleeping for 3 seconds"<<std::endl;
    ros::Duration(3).sleep();
    
    // Initializing service clients
    ros::service::waitForService("computeEnergyFunc",1000);
    ros::service::waitForService("control_points_output", 1000);
    ros::service::waitForService("binary_image_output", 1000);

    ros::ServiceClient energyClient = n.serviceClient<encoderless_vs::energyFuncMsg>("computeEnergyFunc");
    ros::ServiceClient cp_client = n.serviceClient<encoderless_vs::control_points>("control_points_output");

    // Initializing status msg
    std_msgs::Int32 status;
    status.data = 0;
    status_pub.publish(status);

    // Servoing variables
    int window; // Estimation window size
    n.getParam("vsbot/estimation/window", window);

    float rate; // control loop rate
    n.getParam("vsbot/estimation/rate", rate);
    
    float control_rate;
    n.getParam("vsbot/control/rate", control_rate);

    float thresh;
    n.getParam("vsbot/control/thresh",thresh);

    float lam;
    n.getParam("vsbot/control/lam",lam);

    float gain_sm;
    n.getParam("vsbot/control/gain_sm", gain_sm);

    n.getParam("vsbot/shape_control/no_of_features", no_of_features);

    std::vector<float> goal (no_of_features,0);
    n.getParam("shape_controller/goal_features", goal);
    // print_fvector(goal);

    float sat;
    n.getParam("vsbot/shape_control/sat", sat);

    int it = 0; // iterator
    std::vector<float> error (no_of_features,0); //error vector
    float err = 0.0; // error norm
    std_msgs::Float64MultiArray err_msg; // feature error
    // std::cout << "Initialized Servoing Variables" << std::endl;

    // Estimation variables
    float gamma; // learning rate
    n.getParam("vsbot/estimation/gamma", gamma);
    
    float gamma2; // learning rate
    n.getParam("vsbot/estimation/gamma2", gamma2);

    float beta; // threshold for selective Jacobian update
    n.getParam("vsbot/shape_control/beta", beta);
    
    std::vector<float> ds; // change in spline features
    std::vector<float> dr; // change in joint angles
    // float dth; // change in theta
    // theta: angle the line between control pts makes with x-axis

    // std::vector<float> qhat {7,13,-21,17,19,-23,11,-29}; // ,4.7,-13,27,31}; // initial Jacobian matrix as vector
    std::vector<float> qhat {7,13,-21,17,19,-23,11,-29,31,11,3,17};
    // std::vector<float> qhat {7,13,-21,17,19,-23,11,-29,31,11,3,17,1.3,0.9};
    // std::vector<float> qhat {0.07,0.13,-0.21,0.17,0.19,-0.23,0.11,-0.29};
    // std::vector<float> qhat {.7,1.3,-.21,.17,-0.01,-0.5,-0.2,-0.3};
    //  12 elements in qhat(6x2) for 6 elements in features
    // Changed this to 8 elements since only using 4 features now
    
    // float th = 0; // initilaizing theta
    // float old_th = 0; // initialize old th data store

    std::vector<float> dSinitial; // Vector list of shape change vectors
    std::vector<float> dRinitial; // Vector list of position change vectors
    // std::vector<float> dThinitial; // Vector list of changes in theta

    std_msgs::Float64 j1_vel; // joint1 velocity
    std_msgs::Float64 j2_vel; // joint2 velocity
    
    std_msgs::Float64MultiArray ds_msg; // msg to store current dS window
    std_msgs::Float64MultiArray dr_msg; // msg to store current dR window
    // std_msgs::Float32 dth_msg; // msg to store current dTh

    std_msgs::Float64MultiArray control_points; // msg to store control points for current curve

    // Declaring msg for control points service call
    encoderless_vs::control_points cp_msg;
    cp_msg.request.input = 1;

    float t = 1/rate; // time in seconds, used for integrating angular velocity
    // std::cout <<"Initialized estimation variables" << std::endl;


// --------------------------- Initial Estimation -----------------------------    


// command small displacements around initial position
    ros::Rate r{rate};  // Rate for control loop
    std::cout << "Ready to command small displacements" <<std::endl; 
    
    // Obtain initial robot state
    std::vector<float> cur_features(no_of_features, 0);
    cp_client.call(cp_msg);
    control_points.data.clear();
    for(int i = 0; i<no_of_features; i++){
        cur_features[i] = cp_msg.response.cp.data.at(i);
        control_points.data.push_back(cur_features[i]);
    }

    // Obtain th the angle between line and x-axis
    // the line is between feature pt 1 and feature pt n 
    // (n being total number of features pts)
    
    // th = get_cur_th(cur_features[0], cur_features[1], 
                    // cur_features[no_of_features-2], cur_features[no_of_features-1]);
    
    // set old_features to first set of features received
    std::vector<float> old_features = cur_features;
    // old_th = th;

    // Change status msg to initial estimation
    status.data = 1;

    // parameter for generating joint velocities
    float param = 0.6; // resolution for sine vel generation
    
    // Collecting data for estimation window
    while (it < window){

        // Publish sin vel to both joints
        j1_vel.data = 2*sin(param*3.14);
        j2_vel.data = 2*cos(param*3.14);
        
        param = param + 0.1;
        
        // Adding noise to sinusoidal velocity
        j1_vel.data += (2*((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 0.5));
        j2_vel.data += (2*((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 0.5));
        
        j1_pub.publish(j1_vel);
        j2_pub.publish(j2_vel);

        // Obtain current robot state
        cp_client.call(cp_msg);
        control_points.data.clear();
        
        // std::cout<<"# Features: " << no_of_features <<std::endl;

        for(int i = 0; i<no_of_features; i++){
            cur_features[i] = cp_msg.response.cp.data.at(i);
            control_points.data.push_back(cur_features[i]);
        }
        // print_fvector(cur_features);

        // th = get_cur_th(cur_features[0], cur_features[1], 
                        // cur_features[no_of_features-2], cur_features[no_of_features-1]);
        
        // Compute change in state
        // Spline features
        ds.clear();
        for(int i = 0; i<old_features.size(); i++){
            ds.push_back((cur_features[i] - old_features[i]));
        } 

        // th
        // dth = th - old_th;

        //  Joint angle
        dr.clear();
        dr.push_back((j1_vel.data*t));
        dr.push_back((j2_vel.data*t));

        // Update dSinitial and dRinitial
        for(int i = 0; i < no_of_features; i++){
            dSinitial.push_back(ds[i]);
        }
        dRinitial.push_back(dr[0]);
        dRinitial.push_back(dr[1]);

        // Update dThinitial
        // dThinitial.push_back(dth);

        // Update state variables
        old_features = cur_features;
        // std::cout<<"updated state variables"<<std::endl;
        // old_th = th;

        // Publish ds, dr vectors to store
            // Convert to Float64multiarray
            ds_msg.data.clear();
            for(int i = 0; i < no_of_features; i++){
                ds_msg.data.push_back(ds[i]);
            }

            dr_msg.data.clear();
            dr_msg.data.push_back(dr[0]);
            dr_msg.data.push_back(dr[1]);

            // Convert dth
            // dth_msg.data = dth;
            
            // publish
            ds_pub.publish(ds_msg);
            dr_pub.publish(dr_msg);

            // publish dth
            // dth_pub.publish(dth_msg);

        // Publish control points
        cp_pub.publish(control_points);

        // publish status msg
        status_pub.publish(status);

        //Increase iterator 
        // std::cout <<"iterator:" << it <<std::endl;
        it++;

        // Refresh subscriber callbacks
        ros::spinOnce();
        r.sleep();     
    }

    // Commanding 0 velocity to robot 
    j1_vel.data = 0.0;
    j2_vel.data = 0.0;
    j1_pub.publish(j1_vel);
    j2_pub.publish(j2_vel);

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

    // std_msgs::Float32MultiArray dThmsg;
    // dThmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // dThmsg.layout.dim[0].label = "dTh_elements";
    // dThmsg.layout.dim[0].size = dThinitial.size();
    // dThmsg.layout.dim[0].stride = 1;
    // dThmsg.data.clear();

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

    // for(std::vector<float>::iterator itr = dThinitial.begin(); itr!= dThinitial.end(); ++itr){
    //     // std::cout <<*itr<<std::endl;
    //     dThmsg.data.push_back(*itr);
    // }
    
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
        // std::cout<<"size of qhat:"<<qhat.size()<<std::endl;
        for(int i = 0; i<qhat.size(); i++){
            qhat[i] = qhat[i] + qhatdot[i]; // Updating each element of Jacobian
        }
        // std::cout<<"Updated Jacobian vector:";

        // Push updated Jacobian vector to ROS Msg
        qhatmsg.data.clear();
        for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
            qhatmsg.data.push_back(*itr);
        }

        // Publish J value to store
        std_msgs::Float32 J;
        J.data = msg.response.J;
        J_pub.publish(J);
        
        // Increase iterator
        it++;
    }
    std::cout <<"Initial Estimation Completed" << std::endl;

// ----------------------------- Start Servoing ---------------------------------- 
    // err = thresh; // set error norm to threshold to start control loop
    std::cout<<"Entering control loop"<<std::endl;
    
    // Switching to control loop rate
    t = 1/control_rate;
    ros::Rate control_r{control_rate};

    status.data = 2;
    status_pub.publish(status);
    
    // ----------------------- Control Loop for Servoing -----------------------------
    while(!end_flag){    // convergence condition
        // error norm "err" is always positive
        // std::cout<<"Inside control loop"<<std::endl;
        // compute current error & norm
        // print_fvector(cur_features);
        // print_fvector(goal);
        for(int i=0; i<no_of_features;i++){
            error[i] = cur_features[i] - goal[i];
        }
    
        float err_acc = 0; // accumulator vairable for computing error norm
        for(int i=0; i<no_of_features; i++){
            err_acc += error[i]*error[i];
        }
        err = sqrt(err_acc);
        err_acc = 0; // Reset error accumulator
        // std::cout<<" norm:"<<err<<std::endl;

        // Generate velocity
        // Convert qhat vector into matrix format
        Eigen::MatrixXf Qhat(no_of_features,2);

        int row_count = 0;
        int itr = 0;
        
        dr[0] = 0.0; 
        dr[1] = 0.0;

        while(row_count<no_of_features){
            Qhat.row(row_count) << qhat[itr], qhat[itr+1];
            row_count = row_count + 1;
            itr = itr + 2;
        }
        // std::cout<<"Created Jacobian: "<<Qhat<<std::endl;
        
        // Convert error std::vector to Eigen::vector
        // for matrix computations
        Eigen::VectorXf error_vec(no_of_features);
        for(int i=0; i<no_of_features; i++){
            error_vec(i) = error[i];
        }

        // joint velocity Eigen::vector
        Eigen::Vector2f j_vel;

        // std::cout<<"Jacobian: \n"<<Qhat<<std::endl;

        // Closed form solution for linearly independent columns
        // A_inv = (A.transpose()*A).inverse() * A.transpose()
        Eigen::MatrixXf Qhat_inv = (Qhat.transpose()*Qhat).inverse() * Qhat.transpose();
        // std::cout<<"Inverted Jacobian: \n"<<Qhat_inv<<std::endl;
        
        // IBVS control law (Velocity generator)
        /* With Berk 
        P Control 
        j_vel = lam*(Qhat_inv)*(error_vec);
        end of with Berk
        */

        //    Saturated P-controller
        for(int i=0; i<no_of_features; i++){
            if(abs(error_vec(i))>sat)
                error_vec(i) = sat * (error_vec(i)/abs(error_vec(i)));
        }
        
        j_vel = lam*(Qhat_inv)*error_vec;
        
/*        // with Berk Sliding mode control
        Eigen::VectorXf u_sliding_mode(no_of_features);
        Eigen::VectorXf gain_sm_vec(no_of_features);
        gain_sm_vec = gain_sm*(gain_sm_vec.setOnes(no_of_features));
        // gain_sm_mat << 2*gain_sm, gain_sm, gain_sm, gain_sm;

        for(int i=0; i<no_of_features; i++){
            u_sliding_mode[i] = gain_sm_vec[i]*sign(error_vec[i]);
        }
        j_vel = Qhat_inv*u_sliding_mode;
        // end of with Berk Sliding mode control
*/
        // std::cout<<"Computed Velocity"<<std::endl;

        // Publish velocity to robot
        j1_vel.data = j_vel[0];
        j2_vel.data = j_vel[1];
        // std::cout<<"Joint Vel 1: "<<j_vel[0]<<std::endl;
        // std::cout<<"Joint vel 2: "<<j_vel[1]<<std::endl;
        j1_pub.publish(j1_vel);
        j2_pub.publish(j2_vel);
        
        // Get current state of robot
        control_points.data.clear();
        cp_client.call(cp_msg);
        for(int i = 0; i<no_of_features; i++){
            cur_features[i] = cp_msg.response.cp.data.at(i);
            control_points.data.push_back(cur_features[i]);
        }

        // Compute change in state
        ds.clear();
        for(int i=0; i<no_of_features;i++){
            ds.push_back((cur_features[i]-old_features[i]));
        }
        
        dr[0] += j1_vel.data*t;
        dr[1] += j2_vel.data*t;
    
        // Compute shape change magnitude
        float ds_accumulator = 0;
        for(int i = 0; i<no_of_features; i++){
            ds_accumulator += ds[i] * ds[i];
        }
        float ds_norm = sqrt(ds_accumulator);

        if(err > beta){         // Do not update Jacobian if close to the target
        
            // Update sampling windows
            for(int i=0; i<no_of_features;i++){
                dSinitial[i] = ds[i];
            }
            std::rotate(dSinitial.begin(), dSinitial.begin()+no_of_features, dSinitial.end());
            dRinitial[0] = dr[0];
            dRinitial[1] = dr[1];
            std::rotate(dRinitial.begin(), dRinitial.begin()+2, dRinitial.end());
            // Compute Jacobian update with new sampling window
            // converting vectors to ros msg for service
            dSmsg.data.clear();
            for(std::vector<float>::iterator itr = dSinitial.begin(); itr != dSinitial.end(); ++itr){
                dSmsg.data.push_back(*itr);
            }
            dRmsg.data.clear();
            for(std::vector<float>::iterator itr = dRinitial.begin(); itr != dRinitial.end(); ++itr){
                dRmsg.data.push_back(*itr);
            }
            qhatmsg.data.clear();
            for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
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
                qhatmsg.data.push_back(*itr);
            }
            // Update state variables
            old_features = cur_features;
    
            // Publish ds, dr, J, & error vectors to store
            // Convrt to Float64multiarray
            ds_msg.data.clear();
            for(int i=0; i<no_of_features; i++){
                ds_msg.data.push_back(ds[i]);
            }

            dr_msg.data.clear();
            dr_msg.data.push_back(dr[0]);
            dr_msg.data.push_back(dr[1]);

            dr.clear();

            ds_pub.publish(ds_msg);
            dr_pub.publish(dr_msg);
        }

        err_msg.data.clear();
        for(int i = 0; i<no_of_features;i++){
            err_msg.data.push_back(error[i]);
        }

        // publish
        std_msgs::Float32 J;
        J.data = msg.response.J;

        J_pub.publish(J);

        // Publish control points
        cp_pub.publish(control_points);
        
        err_pub.publish(err_msg);
        
        
        // Publish status msg
        status_pub.publish(status);

        // Refresh subscriber callbacks
        ros::spinOnce();
        control_r.sleep();
    }

    // Commanding 0 velocity to robot 
    j1_vel.data = 0.0;
    j2_vel.data = 0.0;
    j1_pub.publish(j1_vel);
    j2_pub.publish(j2_vel);

    std::cout<<"Servoing Complete"<<std::endl;
    status.data = -1;
    status_pub.publish(status);

    // Shutdown
    // Status flag will shutdown record node which is tied to all other nodes
    // This is done so all the recorded files can be closed and saved safely before
    // the nodes shut down

    ros::spin();
    return 0;
}

