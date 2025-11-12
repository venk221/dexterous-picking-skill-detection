#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "origami_control/skeleton_init_estimate.h"
#include "origami_control/skeleton_adaptive_update.h"
#include "origami_vision/curve_msg.h"

#define MAX_PWM -350

ros::ServiceClient adaptive_update_client;
ros::ServiceClient curve_fit_client;
ros::Publisher velocity_pub;
ros::Publisher vel_rec_pub;
ros::Publisher model_error_pub;
ros::Publisher mode_pub;

int num_features = 0;
int num_actuators = 0;
int window_size = 0;
float rate = 0.0;
float adaptive_gain = 0.0;

std::vector<double> qhat;
std::vector<double> cur_features;

bool estimate(origami_control::skeleton_init_estimate::Request &req, 
              origami_control::skeleton_init_estimate::Response &res ){

    // Estimation variables
    int itr = 0;
    ros::spinOnce();
    std::cout<<"Performing initial estimation"<<std::endl;
    
    float param = 0.4;
    float j1_vel = 0.0, j2_vel = 0.0;
    std::vector<float> ds;          // current change in features
    std::vector<float> dr;          // current change in joint position
    std::vector<float> dSinitial;
    std::vector<float> dRinitial;
    
    // Get current features
    origami_vision::curve_msg feature_req_msg;
    
    curve_fit_client.call(feature_req_msg);
    cur_features = feature_req_msg.response.curve_features.data;
    
    std::vector<double> old_features = cur_features;

    // Velocity message
    std_msgs::Int32MultiArray velocity;

    // Control mode msg
    std_msgs::Int32 control_mode;
    control_mode.data = 2;      // velocity control
    mode_pub.publish(control_mode);

    ros::Rate r{rate};
    float t = 1/rate;
    
    while(itr < 3){
        j1_vel = int(MAX_PWM * sin(param));
        j2_vel = int(MAX_PWM * sin(param));

        velocity.data.clear();
        velocity.data.push_back(j1_vel);
        velocity.data.push_back(j2_vel);    
        velocity.data.push_back(j1_vel);
        velocity.data.push_back(j2_vel);

        velocity_pub.publish(velocity);
        // Get current features
        curve_fit_client.call(feature_req_msg);
        cur_features.clear();
        cur_features = feature_req_msg.response.curve_features.data;

        old_features.clear();
        old_features = cur_features;

        param = param + 0.1;
        itr = itr+1;

        ros::spinOnce();
        r.sleep();
    }

    // Reset itr
    itr = 0;
    
    // Command velocities and record state variables to windows
    while(itr<window_size){
        if(itr<window_size){
            j1_vel = int(MAX_PWM * sin(param));
            j2_vel = int(MAX_PWM * sin(param));
        }
        // else{
        //     j1_vel = int(MAX_PWM * cos(param));
        //     j2_vel = int(MAX_PWM * cos(param));
        // }

        velocity.data.clear();
        velocity.data.push_back(j1_vel);
        velocity.data.push_back(j2_vel);    
        velocity.data.push_back(j2_vel);
        velocity.data.push_back(j1_vel);

        velocity_pub.publish(velocity);

        param = param + 0.1;
        itr = itr+1;

        // Get current features
        curve_fit_client.call(feature_req_msg);
        cur_features.clear();
        cur_features = feature_req_msg.response.curve_features.data;

        // Update state variables
        ds.clear();
        // std::cout<<"ds: ";
        for(int i=0; i< num_features; i++){
            ds.push_back(cur_features[i] - old_features[i]);
            // std::cout<< cur_features[i] - old_features[i]<<", ";
        }
        // std::cout<<std::endl;
        // std::cout<<"ds pushed"<<std::endl;
        dr.clear();
        // std::cout<<"dr: ";
        for(int i=0; i<num_actuators; i++){
            dr.push_back(velocity.data.at(i)*t);
            // std::cout<<velocity.data.at(i)*t<<", ";
        }
        // std::cout<<std::endl;

        // Record state variables to window
        for(int i=0; i<num_features; i++){
            dSinitial.push_back(ds[i]);
        }

        for(int i=0; i<num_actuators; i++){
            dRinitial.push_back(dr[i]);
        }


        // Update memory
        old_features.clear();
        old_features = cur_features;
        
        // std::cout<<"iteration#: " <<itr<<std::endl;
        ros::spinOnce();
        r.sleep();
    }

    // Command 0 velocity to the robot
    velocity.data.clear();

    for(int i=0; i<num_actuators; i++){
        velocity.data.push_back(0.0);
    }

    velocity_pub.publish(velocity);
    ros::spinOnce();


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
        // std::cout <<*itr<<std::endl;
        qhatmsg.data.push_back(*itr);
    }
    // std::cout <<"Pushed initial data to ROS msgs"<<std::endl;

    // Compute initial Jacobian estimate
    int it = 0;        // Iterator
    origami_control::skeleton_adaptive_update q_update;
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
        }        // Service request data
        q_update.request.gamma = adaptive_gain;
        q_update.request.it = it;
        q_update.request.dS = dSmsg;
        q_update.request.dR = dRmsg;
        q_update.request.qhat = qhatmsg;

        // std::cout<< std::endl;

        // Publish J value to store
        std_msgs::Float32 model_error;
        model_error.data = q_update.response.J;
        model_error_pub.publish(model_error);
        
        it++;
        ros::spinOnce();
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

    // Read params
    n.getParam("origami_skeleton_vs/window_size", window_size);
    n.getParam("origami_skeleton_vs/control_rate", rate);
    n.getParam("origami_skeleton_vs/init_jacobian", qhat);
    n.getParam("origami_skeleton_vs/gamma", adaptive_gain);
    n.getParam("origami_skeleton_vs/no_of_features", num_features);
    n.getParam("origami_skeleton_vs/no_of_actuators", num_actuators);

    // Service server
    ros::ServiceServer estimation_service = n.advertiseService("origami_skeleton_initial_estimation", estimate);
    std::cout<<"Estimation service initialized"<<std::endl;
    
    // std::cout<<"initial Jacobian size: "<<qhat.size()<<std::endl;
    // std::cout<<"initial Jacobian: ";
    // for(int i = 0; i<qhat.size(); i++){
    //     std::cout<<qhat[i]<<",";
    // }
    // std::cout<<std::endl;
    
    
    // Service clients
    ros::service::waitForService("clothoid_fit_srv", ros::Duration(1000));
    adaptive_update_client  = n.serviceClient<origami_control::skeleton_adaptive_update>("origami_skeleton_adaptive_update");
    curve_fit_client = n.serviceClient<origami_vision::curve_msg>("clothoid_fit_srv");
    
    // Publishers
    velocity_pub = n.advertise<std_msgs::Int32MultiArray>("origami_vs/velocity", 1);
    model_error_pub = n.advertise<std_msgs::Float32>("origami_vs/modelerror", 1);
    mode_pub = n.advertise<std_msgs::Int32>("origami_vs/OMMD_control_mode",1);

    ros::spin();
    return 0;
}