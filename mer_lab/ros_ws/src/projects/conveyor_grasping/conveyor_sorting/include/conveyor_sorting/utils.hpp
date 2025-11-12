#ifndef CONVEYOR_SORTING_UTILS_HPP
#define CONVEYOR_SORTING_UTILS_HPP

#include <string>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/SpawnModel.h>

#include "pugixml.hpp"

bool spawnMesh(ros::ServiceClient& spawnModelClient, const std::string& name, const std::string& urdf, const geometry_msgs::Pose& pose) {
  gazebo_msgs::SpawnModel spawnMsg{};

  spawnMsg.request.model_name = name;
  spawnMsg.request.model_xml = urdf;
  spawnMsg.request.initial_pose = pose;

  return spawnModelClient.call(spawnMsg);
}

// Returns an inertia tensor for a cuboid of given shape
// Returned tensor is valid for unit mass
geometry_msgs::Point computeInertiaTensor(const geometry_msgs::Point& shape, double mass=1.0) {
  geometry_msgs::Point ret;

  const double scaling{mass/12.0};

  ret.x = scaling * (shape.y*shape.y + shape.z*shape.z);
  ret.y = scaling * (shape.x*shape.x + shape.z*shape.z);
  ret.z = scaling * (shape.x*shape.x + shape.y*shape.y);

  return ret;
}

// URDF helpers
std::string createRectangleURDF(const geometry_msgs::Point& size, double mass=1.0, const std::string& jointType="floating", const std::string& color="Gazebo/White") {
  pugi::xml_document doc;

  // String representing box size
  std::ostringstream oss;
  oss << size.x << ' ' << size.y << ' ' << size.z;

  // Main robot node
  auto robotNode{doc.append_child("robot")};
  robotNode.append_attribute("name") = "autogen_rectangle";

  // First link is just world
  robotNode.append_child("link").append_attribute("name") = "world";
  // Second link has all the information
  auto linkNode{robotNode.append_child("link")};
  linkNode.append_attribute("name") = "main";

  // Main link node children
  auto visualNode{linkNode.append_child("visual")};
  auto collisionNode{linkNode.append_child("collision")};
  auto inertialNode{linkNode.append_child("inertial")};

  // Setup visual aspects
  visualNode.append_child("geometry").append_child("box").append_attribute("size") = oss.str().c_str();
  // Setup collision aspects
  collisionNode.append_child("geometry").append_child("box").append_attribute("size") = oss.str().c_str();
  // Setup inertial aspects
  geometry_msgs::Point inertiaTensor{computeInertiaTensor(size, mass)};
  inertialNode.append_child("mass").append_attribute("value") = std::to_string(mass).c_str();
  auto inertiaTensorNode{inertialNode.append_child("inertia")};
  inertiaTensorNode.append_attribute("ixx") = std::to_string(inertiaTensor.x).c_str();
  inertiaTensorNode.append_attribute("ixy") = "0.0";
  inertiaTensorNode.append_attribute("ixz") = "0.0";
  inertiaTensorNode.append_attribute("iyy") = std::to_string(inertiaTensor.y).c_str();
  inertiaTensorNode.append_attribute("iyz") = "0.0";
  inertiaTensorNode.append_attribute("izz") = std::to_string(inertiaTensor.z).c_str();

  // Setup the one joint
  auto jointNode{robotNode.append_child("joint")};
  jointNode.append_attribute("name") = "fixed";
  jointNode.append_attribute("type") = jointType.c_str();
  auto originNode{jointNode.append_child("origin")};
  originNode.append_attribute("rpy") = "0.0 0.0 0.0";
  originNode.append_attribute("xyz") = "0.0 0.0 0.0";
  jointNode.append_child("parent").append_attribute("link") = "world";
  jointNode.append_child("child").append_attribute("link") = "main";

  // Setup gazebo attributes
  auto gazeboNode{robotNode.append_child("gazebo")};
  gazeboNode.append_attribute("reference") = "main";
  gazeboNode.append_child("material").append_child(pugi::node_pcdata).set_value(color.c_str());

  // Setup physics/contact configuration
  // auto velocityDecayNode{gazeboNode.append_child("velocity_decay")};
  // velocityDecayNode.append_child("linear").append_child(pugi::node_pcdata).set_value(std::to_string(10000).c_str());
  // velocityDecayNode.append_child("angular").append_child(pugi::node_pcdata).set_value(std::to_string(10000).c_str());
  // auto gazeboCollisionNode{gazeboNode.append_child("collision")};
  // gazeboCollisionNode.append_attribute("name") = "collision";
  // auto surfaceNode{gazeboCollisionNode.append_child("surface")};
  // auto fOdeNode{surfaceNode.append_child("friction").append_child("ode")};
  // fOdeNode.append_child("mu").append_child(pugi::node_pcdata).set_value(std::to_string(20.0).c_str());
  // fOdeNode.append_child("mu2").append_child(pugi::node_pcdata).set_value(std::to_string(20.0).c_str());
  // auto sOdeNode{surfaceNode.append_child("contact").append_child("ode")};
  // sOdeNode.append_child("kp").append_child(pugi::node_pcdata).set_value(std::to_string(1e6).c_str());
  // sOdeNode.append_child("kd").append_child(pugi::node_pcdata).set_value(std::to_string(1e1).c_str());

  // Convert to string
  std::ostringstream docSS;
  doc.save(docSS);

  return docSS.str();
}

#endif
