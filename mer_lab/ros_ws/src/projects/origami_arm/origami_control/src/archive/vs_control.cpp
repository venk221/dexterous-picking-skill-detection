#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Bool.h"
#include "origami_control/SingleModuleJacobian.h"

#define VELOCITY_CONTROL 2
#define POSITION_CONTROL 1
#define INITIALIZE 0

std::vector<float> cur_ee_pose;     // EE marker pose in image frame
std::vector<float> cur_base_pose;   // Base marker pose in image frame

bool end_flag = false;


void pixelPose(const std_msgs::Float64MultiArray &msg){
    // Update ee marker
    cur_ee_pose.clear();
    cur_ee_pose.push_back(msg.data.at(0));
    cur_ee_pose.push_back(msg.data.at(1));
    std::cout<<"ee marker: " <<msg.data.at(0)<<", "<<msg.data.at(1)<<std::endl;

    // Update base marker
    cur_base_pose.clear();
    cur_base_pose.push_back(msg.data.at(2));
    cur_base_pose.push_back(msg.data.at(3));
    std::cout<<"base marker: " <<msg.data.at(2)<<", "<<msg.data.at(3)<<std::endl;
    std::cout<<"theta: " <<msg.data.at(4)<<", "<<msg.data.at(5)<<std::endl;
}


void endFlag(const std_msgs::Bool &msg){
    // Update end_flag
    end_flag = msg.data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller node");
    ros::NodeHandle n;

    // Initialize subscribers
    ros::Subscriber pose_sub = n.subscribe("origami_vs/aruco/pose", 1, pixelPose);
    ros::Subscriber end_flag_sub = n.subscribe("origami_vs/end_flag", 1, endFlag);

    // Initialize publishers
    ros::Publisher vel_pub = n.advertise<std_msgs::UInt8MultiArray>("origami_vs/velocity", 1);
    ros::Publisher error_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/error", 1);
    ros::Publisher start_record = n.advertise<std_msgs::Bool>("origami_vs/start_record",1);
    ros::Publisher vel_rec_pub = n.advertise<std_msgs::Float64MultiArray>("origami_vs/velocity_log", 1);

    // Service Clients
    ros::service::waitForService("single_module_jacobian", ros::Duration(1000));
    ros::ServiceClient jac_client = n.serviceClient<origami_control::SingleModuleJacobian>("single_module_jacobian");
    origami_control::SingleModuleJacobian  jacobian_msg;

    // Visual Servoing variables
    std::vector<float> Jr;      // Robot Jacobian
    
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

    int step = 0;
    std::vector<float> cur_jacobian;
    
    // TO DO: move this to a separate node and write the measured value to YAML
    // Initialize robot by setting the cable lengths to 0

    // Prompt user for measured value  of tendon length
    
    // Read end-effector marker position

    // Compute transformation between robot end-effector and marker
    std::cout<<"sleeping for 5 seconds"<<std::endl;
    ros::Duration(5).sleep();
    
    // ----------------- Begin Control loop --------------------
    std::cout<<"Entering control loop"<<std::endl;
    
    ros::Rate r{rate};
    ros::spinOnce();
    // Declaring eigen objects for computing velocities
    Eigen::Vector3f e;
    Eigen::Vector3f v;

    std_msgs::Bool record_flag;
    record_flag.data = true;
    start_record.publish(record_flag);

    while(!end_flag){

    // Compute current error
        error.clear();
        error.push_back(0.0);
        for(int i = 0; i<cur_ee_pose.size(); i++){
            error.push_back(cur_ee_pose.at(i) - goal.at(i));
        }

    // Get current Jacobian
        if(jac_client.call(jacobian_msg)){
            step = jacobian_msg.response.step;
            for(int i = 0; i< (step*step); i++){
                cur_jacobian.push_back(jacobian_msg.response.jv.at(i));     // jv = velocity Jacobian
            }
        }
        Eigen::MatrixXf J(step,step);
        // Sat P-Control law
        // v = -lam * J^-1 * sat(e)
        
        // Convert vectors to Eigen objects for vel computation
        
        // Jacobian
        int row_count = 0;
        Eigen::VectorXf jac_data(step);

        while(row_count < step){
            for(int i=0; i< step; i++){
                jac_data[i] = cur_jacobian[row_count*step + i];
            }
            // std::cout<<jac_data<<std::endl;
            J.row(row_count) << jac_data.transpose();
            // std::cout<<"Jacobian row filled"<<std::endl;
            row_count = row_count +1;
        }

        // Error
        for(int i=0; i<3; i++){
            e[i] = error.at(i);
        }

        // Invert Jacobian
        Eigen::MatrixXf J_inv = J.inverse();
        // std::cout <<"Jac: " << J_inv <<std::endl;
        // Generate actuator velocities
        v = -lam * J_inv * e;
        // v[0] = v[2];        // constraining to planar case
        float temp = v[0];  // WHY? change axes?
        v[0] = v[2];
        v[2] = v[1];
        v[1] = temp;        
        // Command actuator velocities
        std_msgs::UInt8MultiArray vel_msg;
        
        // velocity direction bit
        for(int i=0; i<step; i++){
            if(v[i]/abs(v[i]) >0){
                vel_msg.data.push_back(0);      // Positive velocity (Extend)
            }
            else{
                vel_msg.data.push_back(1);      // Negative velocity (contract)
            }
            
        // velocity magnitude bit
            vel_msg.data.push_back(uint(v[i]));         
        }

        // Velcity control mode bit
        vel_msg.data.push_back(VELOCITY_CONTROL);  

        vel_pub.publish(vel_msg);

        // Data logging
        // Publish the error
        std_msgs::Float64MultiArray error_msg;
        std_msgs::Float64MultiArray vel_log;

        // Pushback velocities to log
        for(int i=0; i<3; i++){
            vel_log.data.push_back(v[i]);
        }
        // Pushback errors to log
        for(std::vector<float>::iterator itr = error.begin(); itr!=error.end(); ++itr){
            error_msg.data.push_back(*itr);
        }

        vel_rec_pub.publish(vel_log);
        error_pub.publish(error_msg);

        // Refresh ROS subscribers
        ros::spinOnce();
        r.sleep();
    }

    std::cout<<"Servoing Complete!"<<std::endl;

    // Publish 0 velocity

    
    // ros::spin();
    return 0;
}
