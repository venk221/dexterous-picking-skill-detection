/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 @mainpage
   Desc: GazeboRosCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
*/

#include "gazebo_bb_camera_plugin.hh"

#include <Eigen/Dense>

#include <OgreVector4.h>

#include <string>
#include <regex>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/rendering/rendering.hh>

#include "conveyor_sim/MetaImage.h"
#include "conveyor_sim/Box.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(BoundingBoxCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
BoundingBoxCamera::BoundingBoxCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
BoundingBoxCamera::~BoundingBoxCamera()
{
  ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
}

void BoundingBoxCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  gzdbg << "Loading Bounding Box Plugin..." << std::endl;
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into BoundingBoxCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  // Load regex filter
  if(!_sdf->HasElement("objects"))
    gzerr << "No objects specified for bounding box calculation, doing nothing\n";
  else {
    sdf::ElementPtr objectList = _sdf->GetElement("objects");
    sdf::ElementPtr categoryLabel = objectList->GetElement("label");
    while(categoryLabel) {
      if(!categoryLabel->HasElement("name") || !categoryLabel->HasElement("regex")) {
	gzerr << "Label missing either name or regex!\n";
	categoryLabel = objectList->GetNextElement("label");
	continue;
      }
      gzdbg << "Found regex " << categoryLabel->Get<std::string>("regex") << '\n';
      categories.push_back({categoryLabel->Get<std::string>("name"),
			    std::regex(categoryLabel->Get<std::string>("regex"))});
      categoryLabel = categoryLabel->GetNextElement("label");
    }
  }
  // if(!_sdf->HasElement("obj_regx"))
  //   gzerr << "No conveyor speed specified\n";
  // this->regex_filter = _sdf->GetElement("obj_regx")->Get<std::string>();
  // this->regex = std::regex(this->regex_filter);
  // gzdbg << "Creating bounding boxes for objects given by " << this->regex_filter << '\n';

  GazeboRosCameraUtils::Load(_parent, _sdf);
  assigned = false;
}

  // Method for projecting
  Eigen::Vector4d BoundingBoxCamera::projectToFrame(const ignition::math::Vector3d& pos) {
  auto ignWorldPose = ignition::math::Matrix4d(this->camera->WorldPose());
  auto ignProjPose = ignition::math::Matrix4d(this->camera->ProjectionMatrix());
  // Swap columns to align with opengl matrix
  Eigen::Matrix4d worldPose;
  Eigen::Vector4d objPos;
  Eigen::Matrix4d projPose;
  Eigen::Vector4d objCamPos;
  worldPose <<
    ignWorldPose(0, 1), -ignWorldPose(0, 2), ignWorldPose(0, 0), ignWorldPose(0, 3),
    ignWorldPose(1, 1), -ignWorldPose(1, 2), ignWorldPose(1, 0), ignWorldPose(1, 3),
    ignWorldPose(2, 1), -ignWorldPose(2, 2), ignWorldPose(2, 0), ignWorldPose(2, 3),
    0,               0,                   0,                     1;
  objPos << pos.X(), pos.Y(), pos.Z(), 1.0;
  projPose <<
    ignProjPose(0, 0), ignProjPose(0, 1), ignProjPose(0, 2), ignProjPose(0, 3),
    ignProjPose(1, 0), ignProjPose(1, 1), ignProjPose(1, 2), ignProjPose(1, 3),
    ignProjPose(2, 0), ignProjPose(2, 1), ignProjPose(2, 2), ignProjPose(2, 3),
    ignProjPose(3, 0), ignProjPose(3, 1), ignProjPose(3, 2), ignProjPose(3, 3);
  objCamPos = projPose * worldPose.inverse() * objPos;
  objCamPos[0] /= objCamPos[3];
  objCamPos[1] /= objCamPos[3];
  objCamPos[2] /= objCamPos[3];

  return objCamPos;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void BoundingBoxCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if(!assigned) {
    miPub = this->rosnode_->advertise<conveyor_sim::MetaImage>("/bounding_box", 1);
    assigned = true;
  }
  conveyor_sim::MetaImage img;
  common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      if (sensor_update_time < this->last_update_time_)
      {
          ROS_WARN_NAMED("camera", "Negative sensor update time difference detected.");
          this->last_update_time_ = sensor_update_time;
      }

      // OnNewFrame is triggered at the gazebo sensor <update_rate>
      // while there is also a plugin <updateRate> that can throttle the
      // rate down further (but then why not reduce the sensor rate?
      // what is the use case?).
      // Setting the <updateRate> to zero will make this plugin
      // update at the gazebo sensor <update_rate>, update_period_ will be
      // zero and the conditional always will be true.
      if (sensor_update_time - this->last_update_time_ >= this->update_period_)
      {
        this->PutCameraData(_image, sensor_update_time);
        this->PublishCameraInfo(sensor_update_time);
        this->last_update_time_ = sensor_update_time;
	img.image = this->image_msg_;
	ignition::math::Vector3d center;
	for(physics::ModelPtr mptr : this->world_->Models()) {
	  if(!mptr)		// Null pointer
	    continue;
	  // Determine match and label
	  int index = -1;
	  for(int i = 0; i < categories.size(); ++i)
	    if(std::regex_match(mptr->GetName(), categories[i].second))
	      index = i;
	  if(index == -1)	// No match
	    continue;

	  // Generate bounding box and publish data
	  auto bb = mptr->BoundingBox();
	  center = bb.Center();
	  if(!gazebo::rendering::get_scene()->GetVisual(mptr->GetName()))
	    continue;		// Null pointer

	  gzdbg << "Center: " << center << '\n';
	  gzdbg << "Lengths:\n";
	  gzdbg << bb.XLength() << ", " << bb.YLength() << ", " << bb.ZLength() << '\n';
	  gzdbg << "Rotation: " << gazebo::rendering::get_scene()->GetVisual(mptr->GetName())->Rotation();
	  // Create corners, taking into account center of object
	  std::vector<ignition::math::Vector3d> bbCorners{{center.X() - bb.XLength()/2, center.Y() - bb.YLength()/2, center.Z() - bb.ZLength()/2}, // Bottom face
							  {center.X() + bb.XLength()/2, center.Y() - bb.YLength()/2, center.Z() - bb.ZLength()/2},
							  {center.X() - bb.XLength()/2, center.Y() + bb.YLength()/2, center.Z() - bb.ZLength()/2},
							  {center.X() + bb.XLength()/2, center.Y() + bb.YLength()/2, center.Z() - bb.ZLength()/2},
							    
							  {center.X() - bb.XLength()/2, center.Y() - bb.YLength()/2, center.Z() + bb.ZLength()/2}, // Top face
							  {center.X() + bb.XLength()/2, center.Y() - bb.YLength()/2, center.Z() + bb.ZLength()/2},
							  {center.X() - bb.XLength()/2, center.Y() + bb.YLength()/2, center.Z() + bb.ZLength()/2},
							  {center.X() + bb.XLength()/2, center.Y() + bb.YLength()/2, center.Z() + bb.ZLength()/2},
	  };
	  auto cPos = projectToFrame(center);
	  // Compute bounding box data
	  // These work as long as the image is less than 100k x 100k pixels
	  int tlX=100000, tlY=100000;	// Top-Left corner
	  int brX=0, brY=0;	// Bottom-Right corner
	  // Compute dimensions, calculate relative to center
	  for(const ignition::math::Vector3d& bbPt : bbCorners) {
	    auto framePos = projectToFrame(bbPt);
	    // Update x and y to image resolution
	    framePos[0] = (framePos[0] + 1) / 2.0 * this->width_;
	    framePos[1] = (-framePos[1] + 1) / 2.0 * this->height_;
	    // X
	    if(framePos[0] < tlX)
	      tlX = framePos[0];
	    else if(framePos[0] > brX)
	      brX = framePos[0];

	    // Y
	    if(framePos[1] < tlY)
	      tlY = framePos[1];
	    else if(framePos[1] > brY)
	      brY = framePos[1];
	  }
	  // Now fill in cX, cY, and width/height
	  conveyor_sim::Box b;
	  b.width = brX - tlX; b.height = brY - tlY;
	  b.x = (brX + tlX) / 2; b.y = (brY + tlY) / 2;
	  img.boxes.push_back(b);
	  img.labels.push_back(categories[index].first);
	}

	miPub.publish(img);
      }
    }
  }
}
}
