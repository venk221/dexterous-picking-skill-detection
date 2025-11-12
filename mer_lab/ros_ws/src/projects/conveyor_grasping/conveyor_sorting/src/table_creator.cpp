#include <string>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/SpawnModel.h>

#include "conveyor_sorting/utils.hpp"
#include "conveyor_sorting/config.hpp"

#include <qnd/comm.hpp>

static constexpr char spawnTopic[]{"/gazebo/spawn_urdf_model"};

int main(int argc, char** argv) {
  ros::init(argc, argv, "table_creator_node");
  ros::NodeHandle nh{};

  ConfigLoader cfgLoader{nh};
  if(!cfgLoader.loadParams()) {
    std::cerr << "Failed to load all params\n";
    return 1;
  }

  // Parse args
  geometry_msgs::Point center{cfgLoader.getConfig().table_desc.center};
  geometry_msgs::Point size{cfgLoader.getConfig().table_desc.size};

  // Get service
  ros::ServiceClient spawnClient{*qnd::serviceClient<gazebo_msgs::SpawnModel>(nh, spawnTopic)};

  // Construct URDF
  std::string tableURDF{createRectangleURDF(size, 1000.0, "fixed")};

  // Create pose
  geometry_msgs::Pose tablePose{};
  tablePose.orientation.w = 1.0;
  tablePose.position = center;

  // Spawn table
  if(!spawnMesh(spawnClient, "Table", tableURDF, tablePose)) {
    std::cerr << "Failed to spawn table\n";
    return 2;
  }

  std::cout << "Spawned table\n";

  return 0;
}
