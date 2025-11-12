#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include  <eigen3/Eigen/Dense>

#include "origami_control/skeleton_init_estimate.h"
#include "origami_control/skeleton_adaptive_update.h"


// Global variables
bool end_flag = false;
int num_features = 0;           // number of features
int num_actuators = 0;          // number of directly controlled actuators
std::vector<double> cur_features;
std::vector<double> goal;

void featureCallback(const std_msgs::Float64MultiArray &msg){
    cur_features.clear();
    for(int i=0; i<num_features; i++){
        cur_features.push_back(msg.data.at(i));
    }
}


void endFlagCallback(const std_msgs::Bool &msg){
    end_flag = msg.data;
}


int main(int argc, char **argv){
    // Initialize ROS node
    ros::init(argc, argv, "origami_baseline");
    ros::NodeHandle n;

    // Read parameters
    float lam = 0.0;            // Visual Servoing gain
    float rate = 0.0;           // Control loop frequency
    float gamma = 0.0;          // adaptive gain
    int window_size = 0;        // adaptation size
    float thresh = 0.0;         // update threshold
    float sat = 0.0;            // controller saturation       

    n.getParam("origami_skeleton_vs/lambda", lam);
    n.getParam("origami_skeleton_vs/control_rate", rate);
    n.getParam("origami_skeleton_vs/goal_features", goal);
    n.getParam("origami_skeleton_vs/gamma", gamma);
    n.getParam("origami_skeleton_vs/window_size", window_size);
    n.getParam("origami_skeleton_vs/update_threshold", thresh);
    n.getParam("origami_skeleton_vs/saturation", sat);
    n.getParam("origami_skeleton_vs/no_of_features", num_features);
    n.getParam("origami_skeleton_vs/no_of_actuators", num_actuators);
    
    // Subscribers
    ros::Subscriber feature_sub = n.subscribe("origami_vs/control_points", 1, featureCallback);
    ros::Subscriber end_flag_sub = n.subscribe("origami_vs/end_flag", 1, endFlagCallback);
    
    // Publishers
    // velocity publisher
    ros::Publisher vel_pub = n.advertise<std_msgs::Int32MultiArray>("origami_vs/velocity", 1);
    // velocity logger
    ros::Publisher vel_rec_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/velocity_log", 1);
    // servo error
    ros::Publisher error_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/error", 1);
    // estimation error
    ros::Publisher model_error_pub = n.advertise<std_msgs::Float64>("origami_vs/model_error", 1);
    // record flag
    ros::Publisher start_record = n.advertise<std_msgs::Bool>("origami_vs/start_record",1);
    
    // Wait for services
    ros::service::waitForService("origami_skeleton_initial_estimation", ros::Duration(1000));
    ros::service::waitForService("origami_skeleton_adaptive_update", ros::Duration(1000));

    // Service clients
    ros::ServiceClient estimation_client = n.serviceClient<origami_control::skeleton_init_estimate>("origami_skeleton_initial_estimation");
    ros::ServiceClient update_client = n.serviceClient<origami_control::skeleton_adaptive_update>("origami_skeleton_adaptive_update");
    std::cout<<"setup service clients"<<std::endl;
    
    // Wait for camera
    std::cout<<"sleeping for 20 seconds"<<std::endl;
    ros::Duration(20).sleep();
    
    // Start data logging
    std_msgs::Bool record_flag;
    record_flag.data = true;
    start_record.publish(record_flag);

    // Initial estimation vectors
    std::vector<double> dS;
    std::vector<double> dR;
    std::vector<double> j_hat;

    // Initial estimation
    origami_control::skeleton_init_estimate estimation_msg;
    if(estimation_client.call(estimation_msg)){
        dS = estimation_msg.response.dS.data;
        dR = estimation_msg.response.dR.data;
        j_hat = estimation_msg.response.j_hat.data;
    }

    std::cout<<"Received initial estimate"<<std::endl;

    // ---------------------- Servo phase -----------------------------
    
    // std::cout<<"Size of dS: " << dS.size() <<" Size of dR: " <<dR.size() <<" Size of j_hat: " << j_hat.size()<<std::endl;
    ros::Rate r{rate};

    // Declaring state variables
    std::vector<double> ds;
    std::vector<double> dr;
    
    float t = 1/rate;       // control loop iteration time
    
    // ROS msgs for adaptive update service
    std_msgs::Float64MultiArray dSmsg;
    std_msgs::Float64MultiArray dRmsg;
    std_msgs::Float64MultiArray Jhatmsg;

    // declaring eigen objects for computing velocities
    Eigen::VectorXf e(num_features);
    Eigen::VectorXf v(num_actuators);

    // Velocity message
    std_msgs::Int32MultiArray velocity;
    std_msgs::Float64MultiArray vel_log;

    // Error message
    std_msgs::Float64MultiArray error;
    float error_norm = 0.0;

    // Adaptive update msg
    origami_control::skeleton_adaptive_update adaptive_update_msg;

    // model error msg
    std_msgs::Float64 model_error_msg;
    float model_error = 0.0;

    // Jacobian update object
    std::vector<double> j_hat_dot;

    ros::spinOnce();        // Refresh subscribers before entering control loop
    std::cout<<"Refreshed subscribers"<<std::endl;

    // Memory object for previous position
    std::vector<double> old_features = cur_features;

    // Declare Jacobian matrix
    Eigen::MatrixXf Jhat(num_features, num_actuators);
    
    // --------------------------- adaptive control loop -----------------------------
    std::cout<<"Entering control loop"<<std::endl;

    while(!end_flag){
        // convert j_hat to Eigen library matrix
        int row_count = 0;
        int itr = 0;
        while(row_count<num_features){
            Jhat.row(row_count) << j_hat[itr], j_hat[itr+1], j_hat[itr+2], j_hat[itr+3];
            row_count = row_count + 1;
            itr = itr + num_actuators;
        }

        // compute current feature error
        for(int i=0; i<num_features; i++){
            e[i] = cur_features[i] - goal[i];
        }

        // compute error norm
        double error_temp = 0.0;
        for(int i=0; i<num_features; i++){
            error_temp += e[i]*e[i];
        }
        error_norm = sqrt(error_temp);

        // error saturation
        for(int i=0; i<num_features; i++){
            if (abs(e[i]) > sat){
                e[i] = e[i]/abs(e[i]) * sat;
            }
        }
        // Pseudo inverse of Jhat
        Eigen::MatrixXf Jhat_inv = (Jhat.transpose()*Jhat).inverse() * Jhat.transpose();
        // control law
        v = -lam*Jhat_inv*e;

        // publish velocities
        velocity.data.clear();
        // Module 3
        velocity.data.push_back(int(v[0]));
        velocity.data.push_back(int(v[1]));

        // Module 4
        velocity.data.push_back(int(v[2]));
        velocity.data.push_back(int(v[3]));

        vel_pub.publish(velocity);

        if(error_norm > thresh){            // Adaptive Jacobian update
            
            // compute change in state
            ds.clear();
            for(int i=0; i<num_features; i++){
                ds.push_back(cur_features[i] - old_features[i]);
            }

            dr.clear();
            for(int i=0; i<num_actuators; i++){
                dr.push_back(v[i]*t);
            }

            // modify adaptive window
            for(int i=0; i<num_features; i++){
                dS[i] = ds[i];
            }
            std::rotate(dS.begin(), dS.begin()+num_features, dS.end());
            
            for(int i=0; i<num_actuators; i++){
                dR[i] = dr[i];
            }
            std::rotate(dR.begin(), dR.begin()+num_actuators, dR.end());
            
            // update state
            old_features = cur_features;
                            
            // Push new window to ROS msg
            dSmsg.data.clear();
            for(std::vector<double>::iterator itr = dS.begin(); itr != dS.end(); ++itr){
                // std::cout <<*itr<<std::endl;
                dSmsg.data.push_back(*itr);
            }

            dRmsg.data.clear();
            for(std::vector<double>::iterator itr = dR.begin(); itr != dR.end(); ++itr){
                // std::cout <<*itr<<std::endl;
                dRmsg.data.push_back(*itr);
            }

            Jhatmsg.data.clear();
            for(std::vector<double>::iterator itr = j_hat.begin(); itr != j_hat.end(); ++itr){
                // std::cout <<*itr<<",";
                Jhatmsg.data.push_back(*itr);
            }

            // update Jacobian
            adaptive_update_msg.request.gamma = gamma;
            adaptive_update_msg.request.it = window_size-1;
            adaptive_update_msg.request.dS = dSmsg;
            adaptive_update_msg.request.dR = dRmsg;
            adaptive_update_msg.request.qhat = Jhatmsg;

            if(update_client.call(adaptive_update_msg)){
                model_error = adaptive_update_msg.response.J;
                j_hat_dot.clear();
                j_hat_dot = adaptive_update_msg.response.qhat_dot.data;
            }
            
            for(int i=0; i< j_hat.size(); i++){
                j_hat[i] = j_hat[i] + j_hat_dot[i];
            }
        }

        // data logging
        error.data.clear();
        for(int i=0; i<num_features; i++){
            error.data.push_back(e[i]);
        }
        error_pub.publish(error);
        
        model_error_msg.data = model_error;
        model_error_pub.publish(model_error_msg);
        
        vel_log.data.clear();
        for(int i =0; i<num_actuators; i++){
            vel_log.data.push_back(v[i]);
        }
        vel_rec_pub.publish(vel_log);

        // control loop
        ros::spinOnce();
        r.sleep();
    }

    // Send 0 velocity command to robot
    velocity.data.clear();
    for(int i=0;i<num_actuators;i++){
        velocity.data.push_back(0);
    }
    vel_pub.publish(velocity);

    for(int i =0; i< num_actuators; i++){
        vel_log.data.push_back(0);
    }

    vel_rec_pub.publish(vel_log);
    
    ros::spin();        
    return 0;
}