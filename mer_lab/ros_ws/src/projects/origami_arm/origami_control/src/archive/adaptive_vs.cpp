#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include "origami_control/init_estimate.h"
#include "origami_control/adaptive_update.h"

#define FEATURES 2
#define VELOCITY_CONTROL 2

// Global variables
bool end_flag = false;
std::vector<double> cur_ee_pose;
std::vector<double> cur_base_pose;
std::vector<double> goal;


void poseCallback(const std_msgs::Float64MultiArray &msg){
    // Update ee marker
    cur_ee_pose.clear();
    cur_ee_pose.push_back(msg.data.at(0));
    cur_ee_pose.push_back(msg.data.at(1));

    // Update base marker
    cur_base_pose.clear();
    cur_base_pose.push_back(msg.data.at(2));
    cur_base_pose.push_back(msg.data.at(3));
}

// void goalCallback(const std_msgs::Float64MultiArray &msg){
//     for(int i=0; i< FEATURES; i++){
//         goal.push_back(msg.data.at(i));
//     }
// }

void endFlagCallback(const std_msgs::Bool &msg){
    end_flag = msg.data;;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "origami_baseline");
    ros::NodeHandle n;
    
    // Read parameters
    float lam = 0.0;            // Visual Servoing gain
    float rate = 0.0;           // Control loop frequency
    float gamma = 0.0;          // adaptive gain
    int window_size = 0;        // adaptation size
    float thresh = 0.0;         // update threshold
    float sat = 0.0;            // controller saturation

    n.getParam("origami_adaptive_vs/lambda", lam);
    n.getParam("origami_adaptive_vs/control_rate", rate);
    n.getParam("origami_adaptive_vs/goal_pose", goal);
    n.getParam("origami_adaptive_vs/gamma", gamma);
    n.getParam("origami_adaptive_vs/window_size", window_size);
    n.getParam("origami_adaptive_vs/update_threshold", thresh);
    n.getParam("origami_adaptive_vs/saturation", sat);

    // Subscribers
    ros::Subscriber pose_sub = n.subscribe("origami_vs/aruco/pose", 1, poseCallback);
    // ros::Subscriber goal_sub = n.subscribe("origami_vs/cur_goal", 1, goalCallback);
    ros::Subscriber end_flag_sub = n.subscribe("origami_vs/end_flag", 1, endFlagCallback);

    // Publishers
    ros::Publisher vel_pub = n.advertise<std_msgs::UInt8MultiArray>("origami_vs/velocity", 1);
    ros::Publisher error_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/error", 1);
    ros::Publisher start_record = n.advertise<std_msgs::Bool>("origami_vs/start_record",1);
    ros::Publisher vel_rec_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/velocity_log", 1);
    ros::Publisher model_error_pub = n.advertise<std_msgs::Float64>("origami_vs/model_error", 1);

    // Wait for services
    ros::service::waitForService("origami_initial_estimation", ros::Duration(1000));
    ros::service::waitForService("origami_adaptive_update", ros::Duration(1000));

    // Service client
    ros::ServiceClient estimation_client = n.serviceClient<origami_control::init_estimate>("origami_initial_estimation");
    ros::ServiceClient update_client = n.serviceClient<origami_control::adaptive_update>("origami_adaptive_update");
    
    // Wait for camera
    std::cout<<"sleeping for 5 seconds"<<std::endl;
    ros::Duration(5).sleep();
    
    // Start data logging
    std_msgs::Bool record_flag;
    record_flag.data = true;
    start_record.publish(record_flag);
    
    // Declaring vectors for initial estimation response
    std::vector<double> dS;
    std::vector<double> dR;
    std::vector<double> j_hat;

    // Estimation trajectory
    origami_control::init_estimate estimation_msg;
    if(estimation_client.call(estimation_msg)){
        // Populate response
        dS = estimation_msg.response.dS.data;
        dR = estimation_msg.response.dR.data;
        j_hat = estimation_msg.response.j_hat.data;
    }
    std::cout<<"Received initial estimate"<<std::endl;

    // Servo loop
    std::cout<<"Entering control loop"<<std::endl;
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
    Eigen::Vector2f e;
    Eigen::Vector2f v;

    // Velocity message
    std_msgs::UInt8MultiArray velocity;
    std_msgs::Float64MultiArray vel_log;

    // Error message
    std_msgs::Float64MultiArray error;
    float error_norm = 0.0;

    // Adaptive update msg
    origami_control::adaptive_update adaptive_update_msg;

    // model error msg
    std_msgs::Float64 model_error_msg;
    float model_error = 0.0;

    // Jacobian update object
    std::vector<double> j_hat_dot;

    ros::spinOnce();        // Refresh subscribers before entering control loop
    std::cout<<"Refreshed subscribers"<<std::endl;

    // Memory object for previous position
    std::vector<double> old_ee_pose = cur_ee_pose;
    
    // Declare Jacobian matrix 
    Eigen::MatrixXf Jhat(2,2);
//  ----------------------- adaptive control loop ------------------------------
    while(!end_flag){
        // Convert j_hat to Eigen library matrix
        int row_count = 0;
        int itr = 0;
        while(row_count<FEATURES){
            Jhat.row(row_count) << j_hat[itr], j_hat[itr+1];
            row_count = row_count + 1;
            itr = itr + 2;
        }

        // compute current feature error
        for(int i=0; i<FEATURES; i++){
            e[i] = cur_ee_pose[i] - goal[i];
        }

        // compute error norm
        std::complex<float> err_temp (e[0],e[1]);
        error_norm = sqrt(std::norm(err_temp));

        // error saturation
        for(int i=0; i<FEATURES; i++){
            if (abs(e[i]) > sat){
                e[i] = e[i]/abs(e[i]) * sat;
            }
        }

        // control law
        v = -lam*Jhat.inverse()*e;

        // publish velocities
        velocity.data.clear();
        // Actuator 1
        if(v[0]>0)
            velocity.data.push_back(0);
        else
            velocity.data.push_back(1);
        
        velocity.data.push_back(uint(abs(v[0])));

        // Actuator 2
        if(v[1]>0)
            velocity.data.push_back(0);
        else
            velocity.data.push_back(1);
        
        velocity.data.push_back(uint(abs(v[1])));
        
        // Actuator 3
        if(v[0]>0)
            velocity.data.push_back(0);
        else
            velocity.data.push_back(1);
        
        velocity.data.push_back(uint(abs(v[0])));
        
        // Control mode
        velocity.data.push_back(VELOCITY_CONTROL);
        vel_pub.publish(velocity);

        // compute change in state
        ds.clear();
        ds.push_back((cur_ee_pose[0] - old_ee_pose[0]));
        ds.push_back((cur_ee_pose[1] - old_ee_pose[1]));

        dr.clear();
        dr.push_back((v[0]*t));
        dr.push_back((v[1]*t));
        
        // modify adaptive window
        dS[0] = ds[0];
        dS[1] = ds[1];
        std::rotate(dS.begin(), dS.begin()+2, dS.end());
        
        dR[0] = dr[0];
        dR[1] = dr[1];
        std::rotate(dR.begin(), dR.begin()+2, dR.end());
        
        // update state
        old_ee_pose = cur_ee_pose;

        if(error_norm > thresh){
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
        for(int i=0; i<FEATURES; i++){
            error.data.push_back(e[i]);
        }
        error_pub.publish(error);
        
        model_error_msg.data = model_error;
        model_error_pub.publish(model_error_msg);
        
        vel_log.data.clear();
        vel_log.data.push_back(v[0]);
        vel_log.data.push_back(v[1]);
        vel_log.data.push_back(v[0]);
        vel_rec_pub.publish(vel_log);

        ros::spinOnce();
        r.sleep();
    }

    // Send 0 velocity command to robot
    velocity.data.clear();
    vel_log.data.clear();
    
    for(int i=0; i<6; i++){
        velocity.data.push_back(0);
    }
    velocity.data.push_back(VELOCITY_CONTROL);

    for(int i =0; i< 3; i++){
        vel_log.data.push_back(0);
    }

    vel_pub.publish(velocity);
    vel_rec_pub.publish(vel_log);


    // Shutdown
    ros::shutdown();
    
    ros::spin();        // This is probably not required
    return 0;
}