#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Bool.h"
#include "vs_control/SingleModuleJacobian.h"

std::vector<float> cur_ee_pose;     // EE marker pose in image frame
std::vector<float> cur_base_pose;   // Base marker pose in image frame
std::vector<float> cur_pose;        // EE marker pose w.r.t. base marker

bool end_flag = false;

void relativePose(const std_msgs::Float64MultiArray &msg){
    // Update ee marker
    cur_ee_pose.clear();
    cur_ee_pose.push_back(msg.data.at(0));
    cur_ee_pose.push_back(msg.data.at(1));

    // Update base marker
    cur_base_pose.clear();
    cur_base_pose.push_back(msg.data.at(2));
    cur_base_pose.push_back(msg.data.at(3));

    // Compute ee pose w.r.t base marker
    cur_pose.clear();
    for(int i=0; i< cur_ee_pose.size();i++){
        cur_pose.push_back(cur_ee_pose.at(i) - cur_base_pose.at(i));
    }
}


void endFlag(const std_msgs::Bool &msg){
    // Update end_flag
    end_flag = msg.data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller node");
    ros::NodeHandle n;

    // Initialize subscribers
    ros::Subscriber pose_sub = n.subscribe("origami_vs/aruco/pose", 1, relativePose);
    ros::Subscriber end_flag = n.subscribe("origami_vs/end_flag", 1, endFlag);

    // Initialize publishers
    ros::Publisher vel_pub = n.advertise<std_msgs::UInt8MultiArray>("origami_vs/velocity", 1);
    ros::Publisher error_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/error", 1);

    // Service Clients
    ros::ServiceClient jac_client = n.serviceClient<vs_control::SingleModuleJacobian>("single_module_jacobian");
    vs_control::SingleModuleJacobian  jacobian_msg;

    // Visual Servoing variables
    std::vector<float> Jr;      // Robot Jacobian
    std::vector<float> Ji;      // Image Jacobian
    
    std::vector<float> goal;    // Image-space target
    std::vector<float> error;   // Image-space Servoing error
    
    float lam = 0.0;            // Visual Servoing gain
    float rate = 0.0;           // Control loop frequency
    
    // Read parameters
    float goal_x = 0.0;
    n.getParam("origami_vs/goal_pose_x", goal_x);
    goal.push_back(goal_x);

    float goal_y = 0.0;
    n.getParam("origami_vs/goal_pose_y", goal_y);
    goal.push_back(goal_y);
    
    n.getParam("origami_vs/lambda", lam);
    n.getParam("origami_vs/control_rate", rate);

    float d, l1, l2, l3;
    int step;
    std::vector<float> cur_jacobian;
    
    // TO DO: move this to a separate node and write the measured value to YAML
    // Initialize robot by setting the cable lengths to 0

    // Prompt user for measured value  of tendon length
    
    // Read end-effector marker position

    // Compute transformation between robot end-effector and marker
    
    // ----------------- Begin Control loop --------------------
    std::cout<<"Entering control loop"<<std::endl;
    
    ros::Rate r{rate};
    
    // Declaring eigen objects for computing velocities
    Eigen::Vector3f e;
    Eigen::Vector3f v;
    Eigen::MatrixXf J;
    

    while(!end_flag){
        // Compute current error
        error.clear();
        for(int i = 0; i<cur_pose.size(); i++){
            error.push_back(cur_pose.at(i) - goal.at(i));
        }
        
        // Service message request for Jacobian
        jacobian_msg.request.d = d;
        jacobian_msg.request.l1 = l1;
        jacobian_msg.request.l2 = l2;
        jacobian_msg.request.l3 = l3;

        // Get current Jacobian
        if(jac_client.call(jacobian_msg)){
            step = jacobian_msg.response.step;
            for(int i = 0; i< (step*step); i++){
                cur_jacobian.push_back(jacobian_msg.response.jv.at(i));
            }
        }
        // Sat P-Control law
        // v = -lam * J^-1 * sat(e)
        
        // Convert vectors to Eigen objects for vel computation
        
        // Jacobian
        int row_count = 0;
        Eigen::VectorXf jac_data(step);

        while(row_count < step){
            for(int i=0; i< step; i++){
                jac_data[i] = cur_jacobian[i];
            }
            J.row(row_count) << jac_data.transpose();
            row_count = row_count +1;
        }

        // Error
        e << 0, error[0], error[1]; 

        // Servoing gain
        Eigen::MatrixXf l = Eigen::MatrixXf::Identity(step, step);

        // Invert Jacobian
        Eigen::MatrixXf J_inv = J.inverse();
        
        // Generate actuator velocities
        v = -l * J_inv * e;
        
        // Command actuator velocities

        // Data logging
        // Publish the error
        std_msgs::Float64MultiArray  error_msg;
        for(std::vector<float>::iterator itr = error.begin(); itr!=error.end(); ++itr){
            error_msg.data.push_back(*itr);
        }
        error_pub.publish(error_msg);

        // Refresh ROS subscribers
        ros::spinOnce();
        r.sleep();
    }

    std::cout<<"Servoing Complete!"<<std::endl;

    // Publish 0 velocity

    
    ros::spin();
    return 0;
}