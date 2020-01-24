#ifndef PC_PUB_SUB_H_
#define PC_PUB_SUB_H_

#include "ros/ros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CompressedImage.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

#include <pcl_ros/point_cloud.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/intersections.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <opencv2/opencv.hpp>

#include <vector>

using namespace std;

class PC_PUB_SUB 
{
	public:

		int nContours = 0;

		PC_PUB_SUB ( 
			ros::NodeHandle& nodeHandle, string pointcloud_sub_topic, 
			string filtered_pointcloud_pub_topic, 
			string closest_point_distance_pub_topic,
			string closest_point_base_distance_pub_topic,
			string mask_sub_topic );

		virtual ~PC_PUB_SUB();

		void registerPointCloudSubscriber(string topic);
		void registerImageSubscriber(string topic);
		void registerNContoursSubscriber();
		void registerPointCloudPublisher(string topic);
		void registerDistancePublisher(string topic);
		void registerBaseDistancePublisher(string topic);
		void registerBaseMaxZPublisher();
		void registerClosestPointZPublisher();

		void rosPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_msg);
		void resetNewMeasurementFlag();
		bool newMeasurementRecieved();
		void rosMaskImageCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg);
		void rosNContoursCallback(const std_msgs::Int32::Ptr& ros_msg);

		void publishPointCloud(
			pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, string camera_frame);
		
		void publishDistance(double distance); 
		void publishBaseDistance(double distance); 
		void publishBaseBiggestZ(double z);
		void publishClosestPointZ(double z);
			
		pcl::PointCloud<pcl::PointXYZ>::Ptr getOrganizedCloudPtr();
		vector< vector <int>> getMask();

	private:
		ros::NodeHandle nodeHandle_;
		ros::Subscriber sub_pc2_;
		ros::Subscriber sub_mask_;
		ros::Subscriber sub_nContours_;
		ros::Publisher pub_pc2_;
		ros::Publisher pub_distance_;
		ros::Publisher pub_base_distance_;
		ros::Publisher pub_base_z_max_;
		ros::Publisher pub_closest_point_z_;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr organizedCloudPtr;
		vector< vector <int>> mask;
		double closest_point_distance;
		bool _newMeasurement = false;
};

#endif /* PC_PUB_SUB_H_ */
