#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "controllers/jacobian.h"

// Service server returning robot Jacobian

// Global objects
int no_of_actuators = 0;
int rate = 0;
double l1 = 0;
double l2 = 0;    // Compute Jacobian -- add to yaml
    // Link 1 = 0.5 - 0.05
    // Link 2 = 0.5 + 0.1
std::vector<float> joint_positions;

// Callbacks
void joint_states_callback(const sensor_msgs::JointState &msg){
        joint_positions.clear();
    for (int i = 0; i<no_of_actuators; i++){
        joint_positions.push_back(msg.position.at(i));
    }
}

bool compute_jacobian(controllers::jacobian::Request &req, controllers::jacobian::Response & res){
    // std::cout<<"computing Jacobian"<<std::endl;
    // Compute Jacobian
    float th1 = joint_positions[0];
    float th2 = joint_positions[1];
    // std::cout<<"joints"<<std::endl;
    // std::vector<double> robot_jacobian = {-l1*sin(th1)-l2*sin(th1+th2),-l2*sin(th1+th2), 
                                        //    l1*cos(th1)+l2*cos(th1+th2), l2*cos(th1+th2)};
    std::vector<double> robot_jacobian = {(l1*cos(th1))+(l2*cos(th1+th2)), (l2*cos(th1+th2)),
                                          (l1*sin(th1))+(l2*sin(th1+th2)), (l2*sin(th1+th2))};
    std_msgs::Float64MultiArray robot_jacobian_msg;
    for(int i=0; i<(robot_jacobian.size()); i++){
        robot_jacobian_msg.data.push_back(robot_jacobian[i]);
    }

    // Response
    res.J_r = robot_jacobian_msg;
    return true;
}


int main(int argc, char **argv){
    
    // Init ROS node
    ros::init(argc, argv, "two_link_kinematics");
    ros::NodeHandle n;
    
    // Read configuration parameters
    n.getParam("two_link_vs/no_of_actuators", no_of_actuators);
    n.getParam("two_link_vs/control_rate", rate);
    n.getParam("two_link_vs/l1", l1);
    n.getParam("two_link_vs/l2", l2);

    // Subscribers
    ros::Subscriber robot_joint_states = n.subscribe("two_link/joint_states",1, joint_states_callback);
    
    // Service server
    ros::ServiceServer jacobian_service =  n.advertiseService("two_link_jacobian", compute_jacobian);

    ros::spin();
    return 0;
}