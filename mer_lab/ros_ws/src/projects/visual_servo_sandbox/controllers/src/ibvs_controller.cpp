#include <ros/ros.h>
#include "controllers/jacobian.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include <eigen3/Eigen/Dense>

// Global objects
std::vector<float> cur_feature;
std::vector<float> target_feature;
int no_of_features = 0;
int no_of_actuators = 0;
bool end_flag = false;
int rate = 0;
int debug = 0;
float lam = 0.0;
bool servo = false;
bool ff = false;

// Callbacks
void featureCallback(const std_msgs::Float64MultiArray &msg){
    cur_feature.clear();
    for(int i=0; i<no_of_features; i++){
        cur_feature.push_back(msg.data.at(i));
    }
}

void flagCallback(const std_msgs::Bool &msg){
    servo = msg.data;
}

void refCallback(const std_msgs::Float64MultiArray &msg){
    target_feature.clear();
    for(int i=0; i<no_of_features; i++){
        target_feature.push_back(msg.data.at(i));
    }
    servo = true;
}

// Functions
// TO DO Eigen To ROS Msg
// TO DO Eigen To Vector

int main(int argc, char **argv){
    
    // Init ROS node
    ros::init(argc, argv, "image_servo");
    ros::NodeHandle n;

    // Read configuration parameters
    n.getParam("two_link_vs/no_of_features", no_of_features);
    n.getParam("two_link_vs/no_of_actuators", no_of_actuators);
    n.getParam("two_link_vs/servo_gain", lam);
    n.getParam("two_link_vs/control_rate", rate);
    n.getParam("two_link_vs/debugger_mode", debug);
    n.getParam("two_link_vs/feedforward", ff);

    // Subscribers & services
    ros::Subscriber feature_sub = n.subscribe("two_link_vs/features",1,featureCallback);
    ros::Subscriber end_sub = n.subscribe("two_link_vs/servo", 1, flagCallback);
    ros::Subscriber goal_sub = n.subscribe("two_link_vs/reference", 1, refCallback);
    
    // Service client
    ros::service::waitForService("two_link_jacobian", ros::Duration(1000));
    ros::ServiceClient jacobian_client = n.serviceClient<controllers::jacobian>("two_link_jacobian");

    // Publishers
    ros::Publisher vel_pub = n.advertise<std_msgs::Float64MultiArray>("two_link_vs/j_vel",1);
    ros::Publisher error_pub = n.advertise<std_msgs::Float64MultiArray>("two_link_vs/error",1);

    Eigen::VectorXf error(no_of_features);
    Eigen::VectorXf old_target(no_of_features);
    Eigen::VectorXf cur_target(no_of_features);
    Eigen::VectorXf target_dot(no_of_features);
    Eigen::VectorXf v(no_of_actuators);
    Eigen::MatrixXf J_r(no_of_features, no_of_actuators);
    

    std_msgs::Float64MultiArray vel_msg;
    std_msgs::Float64MultiArray error_msg;
    std_msgs::Float64MultiArray jacobian_msg;
    
    controllers::jacobian robot_jacobian;
    ros::Duration(5).sleep();
    while(cur_feature.size() == 0){
        std::cout<<"waiting for features"<<std::endl;
        ros::spinOnce();
    }
    ros::Rate r{rate};
    while(!servo){
        r.sleep();
        ros::spinOnce();
    }
    // Assign old and cur target
    for(int i=0; i<no_of_features; i++){
        old_target[i] = target_feature[i];
        cur_target[i] = target_feature[i];
    }

    // Control loop
    while(servo){
        // Update sensor readings
        ros::spinOnce();
        
        // Compute current error and update cur target
        for(int i=0;i<no_of_features; i++){
            error[i] = cur_feature[i] - target_feature[i]; 
            cur_target[i] = target_feature[i];
        }

        // Get current Jacobian
        if(jacobian_client.call(robot_jacobian)){
            jacobian_msg = robot_jacobian.response.J_r;
        }

        // Convert Jacobian message to Eigen
        int row_count = 0;
        int itr = 0;
        Eigen::VectorXf temp_row(no_of_actuators);
        while(row_count<no_of_features){
            for(int i=0; i<no_of_actuators;i++){
                temp_row[i] = jacobian_msg.data.at(itr);
                itr = itr+1;
            }
            J_r.row(row_count) << temp_row.transpose();
            row_count = row_count + 1;
        }

        // compute target_dot
        target_dot = cur_target - old_target;
        // std::cout<<target_dot<<std::endl;
        // Control law
        if(ff){
            // control law w/ feed forward term for tracking
            v = -lam * J_r.inverse() * error + J_r.inverse()*target_dot;
        }
        else{
            // control law
            v = -lam * J_r.inverse() * error;
        } 
        
        // publish desired velocity
        vel_msg.data.clear();
        for(int i = 0; i<no_of_actuators; i++){
            vel_msg.data.push_back(v[i]);
        }
        vel_pub.publish(vel_msg);

        // Update old target
        old_target = cur_target;

        // Data logging
        error_msg.data.clear();
        for(int i = 0; i<no_of_features; i++){
            error_msg.data.push_back(error[i]);
        }
        error_pub.publish(error_msg);

        r.sleep();
        // ros::spinOnce();
    }

    // Experiment end procedures
    // Publish 0 velocity
    ros::spin();
    return 0;
}