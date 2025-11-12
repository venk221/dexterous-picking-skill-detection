#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <ros_pcl_manip/Downsample.h>

#include <scrap_burning/PCLConcatRequest.h>

sensor_msgs::PointCloud2 _img;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener;

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudType;
typedef CloudType::Ptr CloudPtr;

ros::Publisher* pub;
ros::ServiceClient down_client;

// Helper method to convert data
CloudPtr from_pc2(const sensor_msgs::PointCloud2& pc2) {
  CloudPtr ret_cloud(new CloudType());
  pcl::PCLPointCloud2 temp_cloud;

  // Copy metadata
  temp_cloud.header.stamp = pc2.header.stamp.toNSec() / 1000ull;
  temp_cloud.header.seq = pc2.header.seq;
  temp_cloud.header.frame_id = pc2.header.frame_id;
  temp_cloud.height = pc2.height;
  temp_cloud.width = pc2.width;
  temp_cloud.fields.resize(pc2.fields.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = pc2.fields.begin();
  int i = 0;
  for(; it != pc2.fields.end(); ++i, ++it) {
    temp_cloud.fields[i].name = (*it).name;
    temp_cloud.fields[i].offset = (*it).offset;
    temp_cloud.fields[i].datatype = (*it).datatype;
    temp_cloud.fields[i].count = (*it).count;
  }
  temp_cloud.is_bigendian = pc2.is_bigendian;
  temp_cloud.point_step = pc2.point_step;
  temp_cloud.row_step = pc2.row_step;
  temp_cloud.is_dense = pc2.is_dense;

  // Copy data
  temp_cloud.data = pc2.data;

  // Place into cloud
  pcl::fromPCLPointCloud2(temp_cloud, (*ret_cloud));

  return ret_cloud;
}

sensor_msgs::PointCloud2 to_pc2(CloudPtr in_cloud) {
  sensor_msgs::PointCloud2 ret;
  pcl::PCLPointCloud2 temp_cloud;

  pcl::toPCLPointCloud2((*in_cloud), temp_cloud);

  ret.header.stamp.fromNSec(temp_cloud.header.stamp  *1000ull);
  ret.header.seq = temp_cloud.header.seq;
  ret.header.frame_id = temp_cloud.header.frame_id;
  ret.height = temp_cloud.height;
  ret.width = temp_cloud.width;
  ret.fields.resize(temp_cloud.fields.size());
  std::vector<pcl::PCLPointField>::const_iterator it = temp_cloud.fields.begin();
  int i = 0;
  for(; it != temp_cloud.fields.end(); ++i, ++it) {
    ret.fields[i].name = (*it).name;
    ret.fields[i].offset = (*it).offset;
    ret.fields[i].datatype = (*it).datatype;
    ret.fields[i].count = (*it).count;
  }
  ret.is_bigendian = temp_cloud.is_bigendian;
  ret.point_step = temp_cloud.point_step;
  ret.row_step = temp_cloud.row_step;
  ret.is_dense = temp_cloud.is_dense;
  ret.data.swap(temp_cloud.data);

  return ret;
}

sensor_msgs::PointCloud2 tf_cloud;
CloudPtr concat_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

void imgCallback(const sensor_msgs::PointCloud2::ConstPtr& img) {
  _img = (*img);
}

bool concat_request(scrap_burning::PCLConcatRequest::Request& req,
		    scrap_burning::PCLConcatRequest::Response& res) {
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  ROS_INFO("Received concatenation request");
  while(true) {
    try {
      if(_img.header.frame_id.empty()) {
	ROS_ERROR("Empty frame id\n");
	return false;
      }
      geometry_msgs::TransformStamped tfStamped =
	tfBuffer.lookupTransform("panda_link0", _img.header.frame_id, ros::Time::now(), ros::Duration(1.0));
      tf2::doTransform(_img, tf_cloud, tfStamped);
      break;
    } catch(tf2::TransformException& ex) {
      ROS_WARN_STREAM(ex.what());
      ros::Duration d(1.0);
      d.sleep();
    }
  }

  // Curve has been transformed, begin concatenation
  CloudPtr cloud = from_pc2(tf_cloud);
  CloudPtr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  (*concat_cloud) += (*cloud);

  // Downsample
  ros_pcl_manip::Downsample down;
  down.request.cloud = to_pc2(concat_cloud);
  down.request.size = 0.001;
  down_client.call(down);
  concat_cloud = from_pc2(down.response.cloud);
  pcl::io::savePCDFileASCII ("/tmp/concatCloud.pcd", (*concat_cloud));

  sensor_msgs::PointCloud2 pub_cloud = to_pc2(concat_cloud);
  res.concat_cloud = pub_cloud;
  pub_cloud.header.frame_id = "/panda_link0";
  ROS_INFO("Done processing, going to publish");

  pub->publish(pub_cloud);
  ros::spinOnce();
  ros::spinOnce();

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_concat_node");
  ros::NodeHandle nh;

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  pub = new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>("concat_points", 1));
  std::string depthTopic{"/panda_camera/depth/points"};
  if(!nh.getParam("/scrap/path_follow/depth_topic", depthTopic))
    ROS_WARN_STREAM("Could not load depth topic, using default " << depthTopic);
  ros::Subscriber img_sub = nh.subscribe(depthTopic, 1, imgCallback);
  down_client = nh.serviceClient<ros_pcl_manip::Downsample>("downsample");

  ros::ServiceServer concat_server = nh.advertiseService("concat_server", concat_request);

  while(ros::ok())
    ros::spinOnce();
}
