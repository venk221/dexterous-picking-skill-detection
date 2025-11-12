#include <ros/ros.h>
#include <cmath>

#include "moveit_planner/MoveCart.h"
#include "moveit_planner/MoveJoint.h"

constexpr double PI=3.141592653589;

std::vector<geometry_msgs::Pose> genCircle(const geometry_msgs::Pose &center, double radius, unsigned int steps) {
  std::vector<geometry_msgs::Pose> ret;

  double angleDelta = 2 * PI / steps;
  constexpr static double startAngle = PI / 2;
  constexpr static double endAngle = startAngle + 2 * PI;
  // Start vertically upward
  for(double angle=startAngle; angle <= endAngle; angle += angleDelta) {
    geometry_msgs::Pose pose(center);
    pose.position.y += (cos(angle) * radius);
    pose.position.z += (sin(angle) * radius);
    ret.push_back(pose);
  }

  return ret;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "movement_test_node");
  ros::NodeHandle nh;

  ros::ServiceClient jointClient = nh.serviceClient<moveit_planner::MoveJoint>("move_to_joint_space");
  ros::ServiceClient cartClient  = nh.serviceClient<moveit_planner::MoveCart>("cartesian_move");

  // Move to initial pose
  moveit_planner::MoveJoint initJoint;
  initJoint.request.val = {0.0, 0.0, 0.0, -PI / 2, 0.0, PI / 2, PI / 4};
  initJoint.request.execute = true;
  jointClient.call(initJoint);

  // Move to test cartesian poses
  moveit_planner::MoveCart cartReq;
  cartReq.request.execute = true;
  cartReq.request.scale = 0.5;

  geometry_msgs::Pose pA;	// Current pose
  pA.position.x = 0.55295;
  pA.position.z = 0.52262;
  pA.orientation.x = 1.0;
  geometry_msgs::Pose circleCenter(pA);
  circleCenter.position.z -= 0.1;
  std::vector<geometry_msgs::Pose> circlePoses = genCircle(circleCenter, 0.1, 100);
  // geometry_msgs::Pose pB(pA);
  // pB.position.z -= 0.25;
  // geometry_msgs::Pose pC(pB);
  // pC.position.y += 0.15;
  // geometry_msgs::Pose pD(pC);
  // pD.position.z += 0.2;

  // cartReq.request.val.push_back(pA);
  // cartReq.request.val.push_back(pB);
  // cartReq.request.val.push_back(pC);
  // cartReq.request.val.push_back(pD);
  cartReq.request.val = circlePoses;
  cartClient.call(cartReq);

  jointClient.call(initJoint);

  return 0;
}
