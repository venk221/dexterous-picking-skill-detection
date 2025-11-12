#include "ros/ros.h"
#include "rosbag/bag.h"
#include <time.h>
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

std_msgs::Float64MultiArray cp;
// std_msgs::Float64MultiArray cp_y;
sensor_msgs::Image img;

void rec_cp(const std_msgs::Float64MultiArray::ConstPtr& msg){
    cp.data = msg->data;
    // cp_y.data = msg->data.at(3);
}

void rec_img(const sensor_msgs::Image image){

    img = image;

}


int main(int argc, char** argv){
    rosbag::Bag bag;
    bag.open("/home/janch-ros/lab_catkin_ws/mer_lab/ros_ws/src/projects/encoderless_vs/bags/record_bag.bag", rosbag::bagmode::Write);

    ros::init(argc, argv, "record_util");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("vsbot/control_points",1,rec_cp);
    ros::Subscriber sub2 = n.subscribe("/vsbot/binary_image",1,rec_img);
    ros::Rate r{30};
    ros::Time t_start = ros::Time::now();
	// ros::Time time_beginning(0.001);
    while(ros::ok()){
        // ros::Time timeStamp = ros::Time::now();
        ros::Time timeStamp = ros::TIME_MIN + (ros::Time::now()-t_start);
        // std::cout << timeStamp << std::endl;
        bag.write("cp", timeStamp , cp);
        bag.write("image",timeStamp, img);

        ros::spinOnce();
        r.sleep();
    }

    bag.close();

    ros::spin();
    return 0;
}