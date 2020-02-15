#ifndef WALL_DETECTION_H
#define WALL_DETECTION_H

#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "Util.h"
#include <pointcloud_filter/WallDetectionParametersConfig.h>
#include <tf2/LinearMath/Quaternion.h>

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
#include <pcl/filters/statistical_outlier_removal.h>
#include <ros/package.h>

namespace detection
{

struct WallDetectionParameters 
{
  static constexpr double MAX_BOX = std::numeric_limits<double>::max();
  static constexpr double MIN_HEIGHT = 1;
  static constexpr double MAX_HEIGHT = 2;
  static constexpr double OUTLIER_MEAN = 50;
  static constexpr double OUTLIER_STDDEV = 1;
  static constexpr double UPSCALE_INCREMENT = 0.05;
  static constexpr double UPSCALE_LIMIT = 0.5;
};

using namespace ros_util;
typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
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
  m_pubWallOdometry = t_nh.advertise<nav_msgs::Odometry>("wall/odometry", 1);
  m_loopTimer = t_nh.createTimer(0.5, 
    &WallDetection::loop_event,
    this
  );

  m_targetWallCloud = boost::make_shared<PCXYZ>();
  wall_from_ply(m_targetWallCloud, "config/zid_tanko_upscale.ply");
}

private:

void do_icp(const PCXYZ::Ptr& t_inputCloud)
{
  if (t_inputCloud->empty())
  {
    ROS_WARN_THROTTLE(5.0, "WallDetection::do_icp - empty cloud");
    return;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(m_targetWallCloud);
  icp.setInputTarget(t_inputCloud);
  auto alignedCloud = boost::make_shared<PCXYZ>();
  icp.align(*alignedCloud);
  ROS_INFO_COND(icp.hasConverged(), "WallDetection::do_icp - ICP converged. Score: [%.2f]", icp.getFitnessScore());
  ROS_FATAL_COND(!icp.hasConverged(), "WallDetection::do_icp - ICP did not converge. :(");

  if (!icp.hasConverged()) {
    return;
  }

  // Do some stuff if converged
  ROS_INFO_STREAM("ICP transformation: " << icp.getFinalTransformation());
  publish_cloud(alignedCloud, m_pubAlignedCloud);
  publish_wall_odometry(icp.getFinalTransformation());
}

void wall_from_ply(PCXYZ::Ptr& t_wallCloud, const std::string& plyPath)
{
  auto wallCloud = boost::make_shared<PCXYZ>();
  std::string path = ros::package::getPath("pointcloud_filter") + "/" + plyPath;
  ROS_INFO("WallDetection - %s", path.c_str());
  if (pcl::io::loadPLYFile(path, *wallCloud) == -1) {
    ROS_FATAL("WallDetection - unable to load whe wall mesh, exiting...");
    throw std::runtime_error("WallDetection - unable to load wall mesh");
  }

  auto wallWithMean = boost::make_shared<PCXYZ>();
  for (const auto& point : wallCloud->points) {
    wallWithMean->points.push_back(pcl::PointXYZ(
        point.x / 1000.0, point.y / 1000.0, point.z / 1000.0
      )
    );
  }

  // Upscale the wall :D
  static constexpr double INITIAL_POSITION = 0.01;
  for (
    double i = INITIAL_POSITION; 
    i < WallDetectionParameters::UPSCALE_LIMIT; 
    i += WallDetectionParameters::UPSCALE_INCREMENT) 
  {
    for (
      double j = INITIAL_POSITION; 
      j < WallDetectionParameters::UPSCALE_LIMIT; 
      j += WallDetectionParameters::UPSCALE_INCREMENT) 
    {
      for (const auto& point : wallCloud->points) {
        wallWithMean->points.push_back(pcl::PointXYZ(
            point.x / 1000.0 + i, point.y / 1000.0 + j, point.z / 1000.0
          )
        );
      }
    }
  }

  // Final wall cloud is the demeaned cloud
  t_wallCloud = demean_cloud(wallWithMean);
  m_lastFoundMapCentroid = Eigen::Vector4f{0,0,0,0};
}

void publish_wall_odometry(const Eigen::Matrix4f& t_transform)
{
  const auto get_translation_x = [&t_transform] () { return t_transform(0, 3); };
  const auto get_translation_y = [&t_transform] () { return t_transform(1, 3); };
  const auto get_translation_z = [&t_transform] () { return t_transform(2, 3); };
  const auto get_transform_quaternion = [&t_transform] () {
    tf::Matrix3x3 mat(
      t_transform(0, 1), t_transform(0, 2), t_transform(0, 3),
      t_transform(1, 1), t_transform(1, 2), t_transform(1, 3),
      t_transform(2, 1), t_transform(2, 2), t_transform(2, 3)
    );
    double roll, pitch, yaw; 
    mat.getRPY(roll, pitch, yaw);
    tf2::Quaternion q(yaw, pitch, roll); 
    return q;    
  };

  nav_msgs::Odometry wallOdom;
  wallOdom.header.stamp = ros::Time::now();
  wallOdom.header.frame_id = ros::this_node::getNamespace() + "/map";
  wallOdom.pose.pose.position.x = get_translation_x() + m_lastFoundMapCentroid.x();
  wallOdom.pose.pose.position.y = get_translation_y() + m_lastFoundMapCentroid.y();
  wallOdom.pose.pose.position.z = get_translation_z() + m_lastFoundMapCentroid.z();
  
  auto q = get_transform_quaternion();
  wallOdom.pose.pose.orientation.x = q.getX();
  wallOdom.pose.pose.orientation.y = q.getY();
  wallOdom.pose.pose.orientation.z = q.getZ();
  wallOdom.pose.pose.orientation.w = q.getW();

  m_pubWallOdometry.publish(wallOdom);
}

PCXYZ::Ptr do_outlier_filtering(const PCXYZ::Ptr& t_inputCloud)
{
	auto filteredCloud = boost::make_shared<PCXYZ>();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (t_inputCloud);
	sor.setMeanK (m_handlerParam->getData().outlier_filter_mean);
	sor.setStddevMulThresh (m_handlerParam->getData().outlier_filter_stddev);
	sor.filter (*filteredCloud);
	return filteredCloud;
}

PCXYZ::Ptr demean_cloud(const PCXYZ::Ptr& t_cloudWithMean)
{
  auto outCloud = boost::make_shared<PCXYZ>();
  pcl::compute3DCentroid(*t_cloudWithMean, m_lastFoundMapCentroid);
  for (const auto& point : t_cloudWithMean->points) {
    outCloud->points.push_back(pcl::PointXYZ(
      point.x - m_lastFoundMapCentroid.x(), 
      point.y - m_lastFoundMapCentroid.y(), 
      point.z - m_lastFoundMapCentroid.z()
    ));
  }
  return outCloud;
}

void initialize_parameters(ros::NodeHandle& t_nh)
{
  DetectionConfig config;
  config.outlier_filter_mean = WallDetectionParameters::OUTLIER_MEAN;
  config.outlier_filter_stddev = WallDetectionParameters::OUTLIER_STDDEV;
  config.min_crop_height = WallDetectionParameters::MIN_HEIGHT;
  config.max_crop_height = WallDetectionParameters::MAX_HEIGHT;
  m_handlerParam = std::make_shared<ParamHandler<DetectionConfig>>(config, "wall_detection");
}

void loop_event(const ros::TimerEvent& /*unused*/)
{
  // Convert from ROS message to pcl
	auto inputCloud = boost::make_shared<PCXYZ>();
	read_input_cloud(inputCloud);
  inputCloud = crop_by_height(inputCloud);
  inputCloud = do_outlier_filtering(inputCloud);
  inputCloud = demean_cloud(inputCloud);
  do_icp(inputCloud);
  publish_cloud(inputCloud, m_pubFilteredCloud);
  publish_cloud(m_targetWallCloud, m_pubTargetCloud);
}

PCXYZ::Ptr crop_by_height(const PCXYZ::ConstPtr& t_inputCloud)
{
  if (t_inputCloud->empty()) {
    return boost::make_shared<PCXYZ>();
  }

  auto newCloud = boost::make_shared<PCXYZ>();
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

inline void read_input_cloud(PCXYZ::Ptr& t_inputCloud)
{
  pcl::fromROSMsg(m_handlerMapCloud.getData(), *t_inputCloud);
}

void publish_cloud(const PCXYZ::Ptr& t_inputCloud, ros::Publisher& t_pub)
{
  auto rosMsg = boost::make_shared<ROSCloud>();
  pcl::toROSMsg(*t_inputCloud, *rosMsg);
  rosMsg->header.frame_id = ros::this_node::getNamespace() + "/map";
  rosMsg->header.stamp = ros::Time::now();
  t_pub.publish(*rosMsg);
}

ros::Timer m_loopTimer;
ros::Publisher m_pubFilteredCloud, m_pubTargetCloud, m_pubAlignedCloud, m_pubWallOdometry;
Eigen::Vector4f m_lastFoundMapCentroid;
TopicHandler<ROSCloud> m_handlerMapCloud;
std::shared_ptr<ParamHandler<DetectionConfig>> m_handlerParam;
PCXYZ::Ptr m_targetWallCloud;

};

}
#endif /* WALL_DETECTION_H */