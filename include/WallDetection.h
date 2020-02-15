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
#include <pcl/PointIndices.h>
#include "pcl/common/impl/centroid.hpp"
#include <pcl/io/ply_io.h>
#include <ros/package.h>

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
  m_pubFilteredCloud = t_nh.advertise<ROSCloud>("filtered_cloud", 1);
  m_pubTargetCloud = t_nh.advertise<ROSCloud>("target_cloud", 1);
  m_pubAlignedCloud = t_nh.advertise<ROSCloud>("aligned_cloud", 1);
  m_loopTimer = t_nh.createTimer(0.5, 
    &WallDetection::loop_event,
    this
  );

  m_targetWallCloud = boost::make_shared<PCLoud>();
  wall_from_ply(m_targetWallCloud, "config/zid_tanko_upscale.ply");
}

private:

void do_icp(const PCLoud::Ptr& t_inputCloud)
{
  if (t_inputCloud->empty())
  {
    ROS_WARN_THROTTLE(5.0, "WallDetection::do_icp - empty cloud");
    return;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(t_inputCloud);
  icp.setInputTarget(m_targetWallCloud);
  auto alignedCloud = boost::make_shared<PCLoud>();
  icp.align(*alignedCloud);
  ROS_INFO_COND(icp.hasConverged(), "WallDetection::do_icp - ICP converged. Score: [%.2f]", icp.getFitnessScore());
  ROS_FATAL_COND(!icp.hasConverged(), "WallDetection::do_icp - ICP did not converge. :(");
  ROS_INFO_STREAM("ICP transformation: " << icp.getFinalTransformation());
  publish_cloud(alignedCloud, m_pubAlignedCloud);
}

void wall_from_ply(PCLoud::Ptr& t_wallCloud, const std::string& plyPath)
{
  auto wallCloud = boost::make_shared<PCLoud>();
  std::string path = ros::package::getPath("pointcloud_filter") + "/" + plyPath;
  ROS_INFO("WallDetection - %s", path.c_str());
  if (pcl::io::loadPLYFile(path, *wallCloud) == -1) {
    ROS_FATAL("WallDetection - unable to load whe wall mesh, exiting...");
    throw std::runtime_error("WallDetection - unable to load wall mesh");
  }

  for (const auto& point : wallCloud->points) {
    t_wallCloud->points.push_back(pcl::PointXYZ(
        point.x / 1000.0, point.y / 1000.0, point.z / 1000.0
      )
    );
  }

  for (double i = 0.01; i < 0.5; i+=0.05) {
  for (double j = 0.01; j < 0.5; j+=0.05) {
    for (const auto& point : wallCloud->points) {
      t_wallCloud->points.push_back(pcl::PointXYZ(
          point.x / 1000.0 + i, point.y / 1000.0 + j, point.z / 1000.0
        )
      );
    }
  }
  }
}

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
  do_icp(croppedCloud);
  publish_cloud(croppedCloud, m_pubFilteredCloud);
  publish_cloud(m_targetWallCloud, m_pubTargetCloud);
}

PCLoud::Ptr filter_by_height(const PCLoud::ConstPtr& t_inputCloud)
{
  if (t_inputCloud->empty()) {
    return boost::make_shared<PCLoud>();
  }

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

void iterative_closest_points()
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
}

inline void read_input_cloud(PCLoud::Ptr& t_inputCloud)
{
  pcl::fromROSMsg(m_handlerMapCloud.getData(), *t_inputCloud);
}

void publish_cloud(const PCLoud::Ptr& t_inputCloud, ros::Publisher& t_pub)
{
  auto rosMsg = boost::make_shared<ROSCloud>();
  pcl::toROSMsg(*t_inputCloud, *rosMsg);
  rosMsg->header.frame_id = ros::this_node::getNamespace() + "/map";
  rosMsg->header.stamp = ros::Time::now();
  t_pub.publish(*rosMsg);
}

ros::Timer m_loopTimer;
ros::Publisher m_pubFilteredCloud, m_pubTargetCloud, m_pubAlignedCloud;
TopicHandler<ROSCloud> m_handlerMapCloud;
std::shared_ptr<ParamHandler<DetectionConfig>> m_handlerParam;
PCLoud::Ptr m_targetWallCloud;

};

}
#endif /* WALL_DETECTION_H */