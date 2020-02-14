#ifndef WALL_DETECTION_H
#define WALL_DETECTION_H

#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "Util.h"
#include <pointcloud_filter/WallDetectionParametersConfig.h>

#include <boost/make_shared.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

namespace detection
{

struct WallDetectionParameters 
{
  static constexpr double MAX_BOX = std::numeric_limits<double>::max();
  static constexpr double MIN_HEIGHT = 1;
  static constexpr double MAX_HEIGHT = 2;
};

using namespace ros_util;
typedef pcl::PointCloud<pcl::PointXYZ> PCLoud;
typedef sensor_msgs::PointCloud2 ROSCloud;
typedef pointcloud_filter::WallDetectionParametersConfig DetectionConfig;

class WallDetection
{
public:
WallDetection(ros::NodeHandle& t_nh) :
  m_handlerMapCloud(t_nh, "submap_cloud")
{
  initialize_parameters(t_nh);
  m_pubTestCloud = t_nh.advertise<ROSCloud>("dummy_cloud", 1);
  m_loopTimer = t_nh.createTimer(0.5, 
    &WallDetection::loop_event,
    this
  );
}

private:

void initialize_parameters(ros::NodeHandle& t_nh)
{
  DetectionConfig config;
  config.min_crop_height = WallDetectionParameters::MIN_HEIGHT;
  config.max_crop_height = WallDetectionParameters::MAX_HEIGHT;
  m_handlerParam = std::make_shared<ParamHandler<DetectionConfig>>(config, "wall_detection");
}

void loop_event(const ros::TimerEvent& /*unused*/)
{
  // Convert from ROS message to pcl
	auto inputCloud = boost::make_shared<PCLoud>();
	read_input_cloud(inputCloud);
  auto croppedCloud = filter_by_height(inputCloud);
  publish_cloud(croppedCloud);
}

PCLoud::Ptr filter_by_height(const PCLoud::ConstPtr& t_inputCloud)
{
  auto newCloud = boost::make_shared<PCLoud>();
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(
    -WallDetectionParameters::MAX_BOX, 
    -WallDetectionParameters::MAX_BOX, 
    m_handlerParam->getData().min_crop_height, 
    0.0
  ));
  boxFilter.setMax(Eigen::Vector4f(
    WallDetectionParameters::MAX_BOX, 
    WallDetectionParameters::MAX_BOX, 
    m_handlerParam->getData().max_crop_height, 
    0.0
  ));
  boxFilter.setInputCloud(t_inputCloud);
  boxFilter.filter(*newCloud);
  return newCloud;
}

inline void read_input_cloud(PCLoud::Ptr& t_inputCloud)
{
  pcl::fromROSMsg(m_handlerMapCloud.getData(), *t_inputCloud);
}

void publish_cloud(const PCLoud::Ptr& t_inputCloud)
{
  auto rosMsg = boost::make_shared<ROSCloud>();
  pcl::toROSMsg(*t_inputCloud, *rosMsg);
  rosMsg->header.frame_id = ros::this_node::getNamespace() + "/map";
  rosMsg->header.stamp = ros::Time::now();
  m_pubTestCloud.publish(*rosMsg);
}

ros::Timer m_loopTimer;
ros::Publisher m_pubTestCloud;
TopicHandler<ROSCloud> m_handlerMapCloud;
std::shared_ptr<ParamHandler<DetectionConfig>> m_handlerParam;
};

}
#endif /* WALL_DETECTION_H */