#include <memory>

#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"
#include "scrap_burning/RecordRequest.h"
#include "scrap_burning/AddToRecord.h"
#include "scrap_burning/FilterPath.h"

#include "octomap_concat_class.cpp"

class OctomapConcatNode {
public:
  OctomapConcatNode(ros::NodeHandle &nh)
    : _nh(nh), _tf() {
    _start  = _nh.advertiseService("begin_octomap", &OctomapConcatNode::_startRec, this);
    _add    = _nh.advertiseService("add_octomap",   &OctomapConcatNode::_addRec, this);
    _filter = _nh.serviceClient<scrap_burning::FilterPath>("path_filter");

    ros::spin();
  }
private:
  ros::NodeHandle _nh;

  TFGetter _tf;
  std::unique_ptr<OctomapConcat> _octomap;

  // Services
  ros::ServiceServer _start;
  ros::ServiceServer _add;
  ros::ServiceClient _filter;

  // Callbacks
  bool _startRec(scrap_burning::RecordRequest::Request  &req,
		 scrap_burning::RecordRequest::Response &res) {
    _octomap = std::make_unique<OctomapConcat>(req.resolution,
					       std::bind(&TFGetter::getTransform, &_tf,
							 req.base_id, req.frame_id),
					       req.filepath);

    return true;
  }
  bool _addRec(scrap_burning::AddToRecord::Request  &req,
	       scrap_burning::AddToRecord::Response &res) {
    _octomap->addCloud(req.cloud);
    _octomap->addLineCloud(req.filtered_cloud);
    _octomap->saveMap();

    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "octomap_concat_node");
  ros::NodeHandle nh;

  OctomapConcatNode ocn(nh);

  return 0;
}
