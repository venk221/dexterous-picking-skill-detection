#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

#include "origami_control/adaptive_update.h"
#include "origami_control/init_estimate.h"

#define MAX_PWM 255
#define FEATURES 2
#define VELOCITY_CONTROL 2

int window_size = 0;
float rate = 0.0;
float adaptive_gain = 0.0;

ros::Publisher vel_pub;
ros::Publisher vel_rec_pub;
ros::Publisher model_error_pub;
ros::ServiceClient adaptive_update_client;

std::vector<float> cur_ee_pose;
std::vector<double> qhat;

void poseCallback(const std_msgs::Float64MultiArray &msg){
    // Update ee marker
    cur_ee_pose.clear();
    cur_ee_pose.push_back(msg.data.at(0));
    cur_ee_pose.push_back(msg.data.at(1));
    // std::cout<<"Pose received"<<std::endl;
}

bool estimate(origami_control::init_estimate::Request &req,
              origami_control::init_estimate::Response &res){
    
    // Estimation variables
    int itr = 0;
    ros::spinOnce();
    std::cout<<"Performing initial estimation"<<std::endl;

    float param = 0.3;
    float j1_vel = 0.0, j2_vel = 0.0;
    std::vector<float> ds;          // current change in ee position
    std::vector<float> dr;          // current change in joint displacement
    std::vector<float> dSinitial;
    std::vector<float> dRinitial;
    std::vector<float> old_ee_pose = cur_ee_pose;
    // std::cout<<"old pose: "<<cur_ee_pose[0]<<", "<<cur_ee_pose[1]<<std::endl;
    
    // Velocity message
    std_msgs::UInt8MultiArray velocity;
    std_msgs::Float64MultiArray vel_log;

    ros::Rate r{rate};
    float t= 1/rate;

    // Command velocities and record state variables to windows
    while(itr<window_size){

        if(itr < window_size/2){
            j1_vel = MAX_PWM * sin(param);
            j2_vel = MAX_PWM * cos(param);
        }
        else{
            j1_vel = MAX_PWM * cos(param);
            j2_vel = MAX_PWM * sin(param);
        }
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
        // std::cout<<"Publishing velocity"<<std::endl;
        vel_pub.publish(velocity);
        // std::cout<<"Published velocity"<<std::endl;
        param = param + 0.1;
        itr = itr+1;
        
        // Update state variables
        ds.clear();
        for(int i=0; i< FEATURES; i++){
            ds.push_back(cur_ee_pose[i] - old_ee_pose[i]);
        }
        // std::cout<<"ds pushed"<<std::endl;
        dr.clear();
        dr.push_back(j1_vel*t);
        dr.push_back(j2_vel*t);

        // Record state variables to window
        for(int i=0; i<FEATURES; i++){
            dSinitial.push_back(ds[i]);
            dRinitial.push_back(dr[i]);
        }
        // log velocity
        vel_log.data.clear();
        vel_log.data.push_back(j1_vel);
        vel_log.data.push_back(j2_vel);
        vel_log.data.push_back(j1_vel);

        vel_rec_pub.publish(vel_log);

        // Update memory
        old_ee_pose.clear();
        old_ee_pose = cur_ee_pose;
        
        // std::cout<<"iteration#: " <<itr<<std::endl;
        ros::spinOnce();
        r.sleep();
    }

    // Command 0 velocity to the robot
    velocity.data.clear();
    for(int i=0; i<6; i++)
        velocity.data.push_back(0);
    velocity.data.push_back(VELOCITY_CONTROL);

    vel_pub.publish(velocity);

    // log velocity
    vel_log.data.clear();
    vel_log.data.push_back(0);
    vel_log.data.push_back(0);
    vel_log.data.push_back(0);

    vel_rec_pub.publish(vel_log);

    std::cout<<"Initial Movements Complete"<<std::endl;

    // Declare ROS msgs for service call
    std_msgs::Float64MultiArray dSmsg;
    dSmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dSmsg.layout.dim[0].label = "dS_elements";
    dSmsg.layout.dim[0].size = dSinitial.size();
    dSmsg.layout.dim[0].stride = 1;
    dSmsg.data.clear();
    
    std_msgs::Float64MultiArray dRmsg;
    dRmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dRmsg.layout.dim[0].label = "dR_elements";
    dRmsg.layout.dim[0].size = dRinitial.size();
    dRmsg.layout.dim[0].stride = 1;
    dRmsg.data.clear();

    std_msgs::Float64MultiArray qhatmsg;
    qhatmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    qhatmsg.layout.dim[0].label = "qhat_elements";
    qhatmsg.layout.dim[0].size = qhat.size();
    qhatmsg.layout.dim[0].stride = 1;
    qhatmsg.data.clear();

    // Push data to ROS msgs
    for(std::vector<float>::iterator itr = dSinitial.begin(); itr != dSinitial.end(); ++itr){
        // std::cout <<*itr<<std::endl;
        dSmsg.data.push_back(*itr);
    }
    
    for(std::vector<float>::iterator itr = dRinitial.begin(); itr != dRinitial.end(); ++itr){
        // std::cout <<*itr<<std::endl;
        dRmsg.data.push_back(*itr);
    }

    for(std::vector<double>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
        std::cout <<*itr<<std::endl;
        qhatmsg.data.push_back(*itr);
    }
    // std::cout <<"Pushed initial data to ROS msgs"<<std::endl;

    // Compute initial Jacobian estimate
    int it = 0;        // Iterator
    origami_control::adaptive_update q_update;
    while(it < window_size){
        // Service request data
        q_update.request.gamma = adaptive_gain;
        q_update.request.it = it;
        q_update.request.dS = dSmsg;
        q_update.request.dR = dRmsg;
        q_update.request.qhat = qhatmsg;

        // call compute energy functional
        adaptive_update_client.call(q_update);

        // Populating service response
        std::vector<double> qhatdot = q_update.response.qhat_dot.data;

        //  Jacobian update
        for(int i = 0; i<qhat.size(); i++){
            qhat[i] = qhat[i] + qhatdot[i]; // Updating each element of Jacobian
        }
        // std::cout<<"Updated Jacobian vector:";

        // Push updated Jacobian vector to ROS Msg
        qhatmsg.data.clear();
        for(std::vector<double>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
            // std::cout <<*itr<<",";
            qhatmsg.data.push_back(*itr);
        }
        // std::cout<< std::endl;

        // Publish J value to store
        std_msgs::Float32 model_error;
        model_error.data = q_update.response.J;
        model_error_pub.publish(model_error);
        
        it++;
    }

    // Populate response msg
    res.dS = dSmsg;
    res.dR = dRmsg;
    res.j_hat = qhatmsg;
    
    std::cout <<"Initial Estimation Completed" << std::endl;
    
    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "initial_estimation_server");
    ros::NodeHandle n;
    
    // Service server
    ros::ServiceServer estimation_service =  n.advertiseService("origami_initial_estimation", estimate);
    std::cout<<"Estimation service initialized"<<std::endl;

    // Service clients
    adaptive_update_client = n.serviceClient<origami_control::adaptive_update>("origami_adaptive_update");

    // Publishers
    vel_pub =  n.advertise<std_msgs::UInt8MultiArray>("origami_vs/velocity",1);
    model_error_pub = n.advertise<std_msgs::Float32>("origami_vs/modelerror", 1);
    vel_rec_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/velocity_log", 1);

    // Read parameters
    n.getParam("origami_adaptive_vs/window_size", window_size);
    n.getParam("origami_adaptive_vs/control_rate", rate);
    n.getParam("origami_adaptive_vs/init_jacobian", qhat);
    n.getParam("origami_adaptive_vs/gamma", adaptive_gain);
    
    // Subscribers
    ros::Subscriber pose_sub = n.subscribe("origami_vs/aruco/pose", 1, poseCallback);
    // std::cout<<"got parameters"<<std::endl;

    ros::spin();
    return 0;
}
