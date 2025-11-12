#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "vs_control/SingleModuleJacobian.h"
#include "vs_control/TwoModuleJacobian.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

float rate = 0.0;
int no_of_features = 0;
bool servo = false;
bool end_flag = false;
int no_of_actuators = 0;
int no_of_modules = 0;
float lam = 0.0;
bool ff_term = false;

std::vector<float> cur_feature;
std::vector<float> cur_target;
std::vector<float> old_target;
std::vector<float> cur_cable_length;

void printVector(std::vector<float> vec){
    for(int i=0; i<vec.size(); i++){
        std::cout<<vec[i]<<", ";
    }
    std::cout<<std::endl;
}

void endFlagCallback(const std_msgs::Bool &msg){
    end_flag = msg.data;
}

void featureCallback(const std_msgs::Float64MultiArray &msg){
    cur_feature.clear();
    for(int i=0; i<no_of_features; i++){
        cur_feature.push_back(msg.data.at(i));
    }
    servo = true;
}

void cableLengthCallback(const std_msgs::Float64MultiArray &msg){
    cur_cable_length.clear();
    for(int i=0; i<no_of_actuators; i++){
        cur_cable_length.push_back(msg.data.at(i));
    }
}

void refCallback(const std_msgs::Float64MultiArray &msg){
    cur_target.clear();
    for(int i=0; i<no_of_features; i++){
        cur_target.push_back(msg.data.at(i));
    }
}

int main(int argc, char **argv){
    
    // Init ROS node
    ros::init(argc, argv, "sim_image_servo");
    ros::NodeHandle n;

    // Read params
    n.getParam("origami_sim/control_rate", rate);
    n.getParam("origami_sim/no_of_features", no_of_features);
    n.getParam("origami_sim/no_of_actuators", no_of_actuators);
    n.getParam("origami_sim/num_of_modules", no_of_modules);
    n.getParam("origami_sim/lam", lam);
    n.getParam("origami_sim/feedforward", ff_term);

    // Subscribers
    ros::Subscriber feature_sub = n.subscribe("origami_vs/features", 1, featureCallback);
    ros::Subscriber goal_sub = n.subscribe("origami_vs/reference", 1, refCallback);
    ros::Subscriber cable_length_sub = n.subscribe("origami_vs/cable_lengths", 1, cableLengthCallback);
    ros::Subscriber end_flag_sub = n.subscribe("origami_vs/end_flag", 1, endFlagCallback);

    // Publishers
    ros::Publisher vel_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/vel", 1);
    ros::Publisher error_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/error", 1);

    // Single module Service client
    ros::service::waitForService("single_module_jacobian", ros::Duration(1000));
    ros::ServiceClient jacobian_client = n.serviceClient<vs_control::SingleModuleJacobian>("single_module_jacobian");
    // Two module Jacobian service client
    // ros::service::waitForService("two_module_jacobian", ros::Duration(1000));
    // ros::ServiceClient jacobian_client = n.serviceClient<vs_control::TwoModuleJacobian>("two_module_jacobian");
    
    vs_control::SingleModuleJacobian jacobian_msg;
    // vs_control::TwoModuleJacobian jacobian_msg;
    
    Eigen::VectorXf error(no_of_features);
    Eigen::VectorXf target_dot(no_of_features);
    Eigen::VectorXf v(no_of_actuators);
    Eigen::MatrixXf J_robot(3,3);
    std::vector<float> cur_jacobian;
    
    float d;
    d = 40;    // 40 mm radius of module
    int step;

    std_msgs::Float64MultiArray vel_msg;
    std_msgs::Float64MultiArray error_msg;  

    ros::Rate r{rate};
    while(!servo){
        ros::spinOnce();
        // std::cout<<"waiting to servo"<<std::endl;
        r.sleep();
    }

    old_target = cur_target;
    std::cout<<"Entering control loop"<<std::endl;
    // Control loop
    while(!end_flag){
        // Refresh subscribers
        ros::spinOnce();

        // Rotating error vector and converting to Eigen object
        error[0] = cur_feature[0] - cur_target[0];
        error[1] = cur_feature[2] - cur_target[2];      // This is 0 
        error[2] = cur_feature[1] - cur_target[1];
        // std::cout<<"error: "<<error<<std::endl;
        
        // Measure target velocity and rotate to same frame as error
        target_dot[0] = cur_target[0] - old_target[0];
        target_dot[1] = cur_target[2] - old_target[2];  // This is 0
        target_dot[2] = cur_target[1] - old_target[1];

        // Request jacobian msg
        jacobian_msg.request.d = d;
        jacobian_msg.request.l1 = cur_cable_length[0];
        jacobian_msg.request.l2 = cur_cable_length[1];
        jacobian_msg.request.l3 = cur_cable_length[2];
        // jacobian_msg.request.l4 = cur_cable_length[3];
        // jacobian_msg.request.l5 = cur_cable_length[4];
        // jacobian_msg.request.l6 = cur_cable_length[5];

        // Jacobian service call
        if(jacobian_client.call(jacobian_msg)){
            step = jacobian_msg.response.step;
            for(int i = 0; i< (step*step); i++){
                cur_jacobian.push_back(jacobian_msg.response.jv.at(i));
            }
        }

        // // Convert Jacobian vector to Eigen object      
        int row_count = 0;
        Eigen::VectorXf jac_row_data(step);

        while(row_count < step){
            for(int i=0; i< step; i++){
                jac_row_data[i] = cur_jacobian[row_count*step+i];
            }
            J_robot.row(row_count) << jac_row_data.transpose();
            row_count = row_count +1;
        }

        // Jacobian inverse
        Eigen::MatrixXf J_robot_inv(3,3);
        J_robot_inv = J_robot.inverse();
        
        // Compute velocity
        if(!ff_term){
            v = -lam*J_robot_inv*error;
        }
        else{
            v = -lam*J_robot_inv*error + J_robot_inv*target_dot;
        }
        
        // Publish data
        vel_msg.data.clear();
        error_msg.data.clear();

        for(int i = 0; i<no_of_actuators*no_of_modules; i++){
            vel_msg.data.push_back(v[i]);
        }


        for(int i = 0; i<no_of_features; i++){
            error_msg.data.push_back(error[i]);
        }

        vel_pub.publish(vel_msg);
        error_pub.publish(error_msg);

        // Update old target
        old_target = cur_target;
        
        r.sleep();
    }

    // Send 0 velocity
    vel_msg.data.clear();
    for(int i = 0; i<no_of_actuators*no_of_modules; i++){
        vel_msg.data.push_back(0.0);
    }

    ros::spin();
    return 0;
}