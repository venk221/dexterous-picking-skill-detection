#include <string>

#include <ros/ros.h>
#include <ros/topic.h>
#include <qnd/cam.hpp>
#include <std_msgs/Float64.h>

#include <opencv2/opencv.hpp>

namespace conveyor_sorting {

  class Scorer {
  public:
    Scorer(ros::NodeHandle& nh, const std::string& scoreTopic, const std::string& scoreCamTopic,
	   ros::Rate r=ros::Rate(60.0))
      : _nh(nh), _scoreTopic(scoreTopic), _scoreImgTopic(scoreCamTopic), _r(r),  _latestScore{-1.0} {
      _nh.subscribe(_scoreTopic, 1, &Scorer::scoreCallback, this);
    }

    void scoreCallback(const std_msgs::Float64& msg) {
      ROS_INFO_STREAM("Received new score");
      // Just update the latest score
      _latestScore = msg.data;
    }

    std::pair<cv::Mat, double> operator()() {
      std::pair<cv::Mat, double> ret{};

      ret.first = qnd::toCV(*ros::topic::waitForMessage<sensor_msgs::Image>(_scoreImgTopic));
      ret.second = ros::topic::waitForMessage<std_msgs::Float64>(_scoreTopic)->data;

      return ret;
    }
  private:
    ros::NodeHandle _nh;
    std::string _scoreTopic;
    std::string _scoreImgTopic;
    ros::Rate _r;

    double _latestScore;
  };

} // conveyor_sorting
