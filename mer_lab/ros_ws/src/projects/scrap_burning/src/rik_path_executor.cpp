#include <math.h>

#include <ros/ros.h>

#include "relaxed_ik/EEPoseGoals.h"
#include "scrap_burning/RelaxedPath.h"

constexpr char PATH_EXEC_TOPIC[]  = "/execute_relaxed_path";
constexpr char RELAXED_IK_TOPIC[] = "/relaxed_ik/ee_pose_goals";

ros::Publisher goalPub;

double mag(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  return sqrt(pow(p1.position.x - p2.position.x, 2) +
	      pow(p1.position.y - p2.position.y, 2) +
	      pow(p1.position.z - p2.position.z, 2));
}

bool path_exec_cb(scrap_burning::RelaxedPath::Request& req,
		  scrap_burning::RelaxedPath::Response& res) {
  auto curPose = req.poses[0];
  relaxed_ik::EEPoseGoals poseGoal;
  poseGoal.header.frame_id = "/panda_link0";
  poseGoal.ee_poses.push_back(curPose);
  goalPub.publish(poseGoal);
  ros::Duration(2.0).sleep();

  for(const auto& pose : req.poses) {
    double dist = mag(pose, curPose);
    ros::Duration(dist / req.vel).sleep();

    poseGoal.ee_poses = std::vector<geometry_msgs::Pose>{pose};
    goalPub.publish(poseGoal);

    curPose = pose;
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rik_path_executor_node");
  ros::NodeHandle nh;

  ros::ServiceServer execSrv = nh.advertiseService(PATH_EXEC_TOPIC, path_exec_cb);
  goalPub = nh.advertise<relaxed_ik::EEPoseGoals>(RELAXED_IK_TOPIC, 1);

  ros::spin();

  return 0;
}
