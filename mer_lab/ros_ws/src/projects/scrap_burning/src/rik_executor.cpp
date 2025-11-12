#include <ros/ros.h>

#include "std_msgs/Float64MultiArray.h"
#include "relaxed_ik/JointAngles.h"

constexpr unsigned long RATE      = 400; // Run at RATE Hz
constexpr char CONTROLLER_TOPIC[] = "/panda/gazebo_joint_position_controller/command";
constexpr char RELAXED_IK_TOPIC[] = "/relaxed_ik/joint_angle_solutions";

// Forward-declare publisher
ros::Publisher posPub;

void jointSolCallback(const relaxed_ik::JointAngles::ConstPtr& msg) {
  // We can just publish the angles member
  posPub.publish(msg->angles);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rik_executor_node");
  ros::NodeHandle nh;

  posPub = nh.advertise<std_msgs::Float64MultiArray>(CONTROLLER_TOPIC, 1);
  ros::Subscriber jointSub = nh.subscribe(RELAXED_IK_TOPIC, 1, jointSolCallback);

  ros::Rate r(RATE);
  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
