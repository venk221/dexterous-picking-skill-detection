#include <memory>
#include <fstream>

#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include "scrap_burning/path.hpp"
#include "scrap_burning/AddTrajWaypoint.h"
#include "scrap_burning/StopRecordingTraj.h"
#include "scrap_burning/StartRecordingTraj.h"
#include "scrap_burning/OutputRecordedTraj.h"

class PathRecorder {
public:
  PathRecorder(ros::NodeHandle &nh)
    : _data(nullptr), _nh(nh) {

    _start = _nh.advertiseService("start_recording_traj", &PathRecorder::cbStartRecording, this);
    _stop = _nh.advertiseService("stop_recording_traj", &PathRecorder::cbStopRecording, this);
    _output = _nh.advertiseService("output_recording_traj", &PathRecorder::cbOutputRecording, this);
    _addWaypoint = _nh.advertiseService("add_traj_waypoint", &PathRecorder::cbAddWaypoint, this);

    while(ros::ok()) {
      if(_data != nullptr) {
	_data->_path.addPoint();

	_data->_r.sleep();
      }
      ros::spinOnce();
    }
  }

  // Callbacks
  bool cbStartRecording(scrap_burning::StartRecordingTraj::Request  &req,
			scrap_burning::StartRecordingTraj::Response &res) {
    _data.reset(new FollowData{
	scrap_burning::Path(req.base, req.target), scrap_burning::Path(req.base, req.target),
        ros::Rate(req.rate)
	  });
    return true;
  }
  bool cbStopRecording(scrap_burning::StopRecordingTraj::Request  &req,
		       scrap_burning::StopRecordingTraj::Response &res) {
    if(_data == nullptr)
      return false;

    _outputRecording(req.filepath);
    _data.reset(nullptr);

    return true;
  }
  bool cbOutputRecording(scrap_burning::OutputRecordedTraj::Request  &req,
			 scrap_burning::OutputRecordedTraj::Response &res) {
    if(_data == nullptr)
      return false;

    _outputRecording(req.filepath);

    return true;
  }
  bool cbAddWaypoint(scrap_burning::AddTrajWaypoint::Request &req,
		     scrap_burning::AddTrajWaypoint::Response &res) {
    if(_data == nullptr)
      return false;
    
    _data->_waypoints.addPoint();

    return true;
  }
private:
  struct FollowData {
    scrap_burning::Path _path;
    scrap_burning::Path _waypoints;
    ros::Rate _r;
  };
  std::unique_ptr<FollowData> _data;
  ros::NodeHandle &_nh;
  
  ros::ServiceServer _start;
  ros::ServiceServer _stop;
  ros::ServiceServer _output;
  ros::ServiceServer _addWaypoint;

  void _outputRecording(const std::string &filepath) const {
    if(filepath.empty() || _data == nullptr)
      return;
    
    std::ofstream ofs(filepath);
    ofs << "Poses:\n" << _data->_path << "Waypoints:\n" << _data->_waypoints;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_recorder_node");
  ros::NodeHandle nh;

  PathRecorder rec(nh);

  return 0;
}
