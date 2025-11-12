#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "ros/package.h"

#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <stdlib.h>

// NOTE: Relies on the scrap being called "scrap" in gazebo to delete it

// Set defaults
std::string to_spawn = "simpleCylinder";
geometry_msgs::Pose target_pose;

int main(int argc, char** argv) {
  ros::init(argc, argv, "scrap_spawn_node");
  ros::NodeHandle nh;

  // Load and setup default values
  target_pose.position.x = 0.75;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.0;
  target_pose.orientation.w = 1.0;

  if(argc > 1)
    to_spawn = std::string(argv[1]);

  // Overrides for position
  // TODO: Implement rotation overrides as well
  if(argc > 2)
    target_pose.position.x = atof(argv[2]);
  if(argc > 3)
    target_pose.position.y = atof(argv[3]);
  if(argc > 4)
    target_pose.position.z = atof(argv[4]);

  ROS_INFO("Arguments: obj_name x y z");

  // Setup the clients to interact with gazebo
  ros::ServiceClient spawnGazeboClient = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_model");
  ros::ServiceClient deleteGazeboClient = nh.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

  // Get the file from the path here
  std::ifstream ifs;
  ifs.open(ros::package::getPath("scrap_burning") + "/scraps/" + to_spawn);
  // Error handling if we could not open it
  if(ifs.fail()) {
    ROS_ERROR("Could not find scrap file");
    return 1;
  }

  // Load the contents of the file at that location
  std::ostringstream sstr;
  sstr << ifs.rdbuf();

  // Delete whatever scrap is in gazebo
  gazebo_msgs::DeleteModel deleteScrapMsg;
  deleteScrapMsg.request.model_name = "scrap";
  deleteGazeboClient.call(deleteScrapMsg);

  // Spawn the model
  gazebo_msgs::SpawnModel spawnScrapMsg;
  spawnScrapMsg.request.model_name = "scrap";
  spawnScrapMsg.request.model_xml = sstr.str();
  spawnScrapMsg.request.initial_pose = target_pose;
  spawnScrapMsg.request.reference_frame = "/world";
  spawnGazeboClient.call(spawnScrapMsg);

  return 0;
}
