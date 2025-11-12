#ifndef CONVEYOR_SORTING_MOVER_HPP
#define CONVEYOR_SORTING_MOVER_HPP

#include <qnd/comm.hpp>
#include <moveit_planner/GetPose.h>
#include <moveit_planner/MovePose.h>
#include <moveit_planner/MoveCart.h>
#include <moveit_planner/MoveJoint.h>
#include <moveit_planner/AddCollision.h>

namespace conveyor_sorting {

  class EmptyMover {
  public:
    EmptyMover() {}

    bool operator()(const geometry_msgs::Pose& pose, bool execute=true) {return true;}
    bool operator()(const std::vector<double>& joints, bool execute=true) {return true;}
  };

  class Mover {
  public:
    Mover(ros::NodeHandle& nh, ros::Duration timeout=qnd::indefinite)
      : _poseClient{}, _jointClient{}, _cartClient{}, _getPoseClient{}, _collClient{} {
      _setupServices(nh, timeout);
    }

    // Different overloads for the movement based on what we are given
    bool operator()(const geometry_msgs::Pose& pose, bool execute=true) {
      moveit_planner::MovePose msg{};
      msg.request.val = pose;
      msg.request.execute = execute;
      // Move by pose
      return _poseClient.call(msg);
    }
    bool operator()(const std::vector<double>& joints, bool execute=true) {
      moveit_planner::MoveJoint msg{};
      msg.request.val = joints;
      msg.request.execute = execute;
      // Move by joint
      return _jointClient.call(msg);
    }
    bool operator()(const std::vector<geometry_msgs::Pose>& poses, bool execute=true) {
      moveit_planner::MoveCart msg{};
      msg.request.val = poses;
      msg.request.scale = 1.0;
      msg.request.execute = execute;
      // Move by cartesian
      return _cartClient.call(msg);
    }

    // Other helpful methods
    bool addCollision(const moveit_msgs::CollisionObject& collObject) {
      moveit_planner::AddCollision msg{};
      msg.request.collObject = collObject;

      return _collClient.call(msg);
    }
  private:
    // Services used with moveit_planner
    ros::ServiceClient _poseClient;
    ros::ServiceClient _jointClient;
    ros::ServiceClient _cartClient;
    ros::ServiceClient _getPoseClient;
    ros::ServiceClient _collClient;

    void _setupServices(ros::NodeHandle& nh, ros::Duration timeout) {
      std::optional<ros::ServiceClient> poseClient{qnd::serviceClient<moveit_planner::MovePose>(nh, "move_to_pose", timeout)};
      std::optional<ros::ServiceClient> jointClient{qnd::serviceClient<moveit_planner::MoveJoint>(nh, "move_to_joint_space", timeout)};
      std::optional<ros::ServiceClient> cartClient{qnd::serviceClient<moveit_planner::MoveCart>(nh, "cartesian_move", timeout)};
      std::optional<ros::ServiceClient> getPoseClient{qnd::serviceClient<moveit_planner::GetPose>(nh, "get_pose", timeout)};
      std::optional<ros::ServiceClient> collClient{qnd::serviceClient<moveit_planner::AddCollision>(nh, "add_collision_object", timeout)};

      if(!poseClient || !jointClient || !cartClient || !getPoseClient || !collClient)
	ROS_FATAL("Failed to obtain one of the necessary clients");
      
      _poseClient = *poseClient;
      _jointClient = *jointClient;
      _cartClient = *cartClient;
      _getPoseClient = *getPoseClient;
      _collClient = *collClient;
    }
  };

} // conveyor_sorting

#endif
