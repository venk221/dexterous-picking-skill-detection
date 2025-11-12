#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include "origami_vision/curve_msg.h"

std::vector<double> cur_features;
bool end_flag = false;

void endFlagCallback(const std_msgs::Bool &msg){
    end_flag = msg.data;
}

int main(int argc, char **argv){
    // Initialize ROS node
    ros::init(argc, argv, "origami_kappa_test");
    ros::NodeHandle n;

    // Declare service
    ros::service::waitForService("clothoid_fit_srv", ros::Duration(1000));
    
    // Read params
    float rate = 0.0;           // Control loop frequency
    n.getParam("origami_skeleton_vs/control_rate", rate);
    
    // Subscriber
    ros::Subscriber end_flag_sub = n.subscribe("origami_vs/end_flag",1 , endFlagCallback);
    
    // Publishers
    // record flag
    ros::Publisher record_var_pub = n.advertise<std_msgs::Int32>("origami_vs/start_record",1);

    // Service client
    ros::ServiceClient feature_client = n.serviceClient<origami_vision::curve_msg>("clothoid_fit_srv");
    
    std_msgs::Int32 record_var;
    record_var.data = 0;
    record_var_pub.publish(record_var);
    

    // Wait for camera
    std::cout<<"sleeping for 20 seconds"<<std::endl;
    ros::Duration(20).sleep();
    ros::spinOnce();

    // Start data logging
    record_var.data = 1;
    record_var_pub.publish(record_var);
    
    // Get current features
    origami_vision::curve_msg feature_req_msg;
    feature_client.call(feature_req_msg);
    ros::spinOnce();

    // Call service in rate loop
    ros::Rate r{rate};
    while(!end_flag){

        feature_client.call(feature_req_msg);
        
        r.sleep();
        ros::spinOnce;
    }

    record_var.data = 3;
    record_var_pub.publish(record_var);
    
    ros::spin();
    return 0;
}