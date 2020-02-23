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
#include "std_msgs/String.h"

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
		int nPatches = 0;
		bool calc_brick_ = false;
		bool calc_patch_ = false;

		PC_PUB_SUB ( 
			ros::NodeHandle& nodeHandle, string pointcloud_sub_topic, 
			string filtered_pointcloud_pub_topic, 
			string closest_point_distance_pub_topic,
			string closest_point_base_distance_pub_topic,
			string mask_sub_topic_brick,
			string mask_sub_topic_patch );

		virtual ~PC_PUB_SUB();

		void registerPointCloudSubscriber(string topic);
		void registerImageSubscriberBrick(string topic);
		void registerImageSubscriberPatch(string topic);
		void registerNContoursSubscriber(string topic);
		void registerNPatchesSubscriber(string topic);
		void registerPointCloudPublisher(string topic);
		void registerDistancePublisher(string topic, string topic_patch);
		void registerBaseDistancePublisher(string topic, string topic_patch);
		void registerBaseMaxZPublisher(string topic, string topic_patch);
		void registerClosestPointZPublisher(string topic, string topic_patch);

		void registerBrickColorSubscriber(string topic);
		void registerSmStateSubscriber(string topic);


		vector <vector <int>> processMaskImage(const cv::Mat image);

		void rosPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_msg);
		void resetNewMeasurementFlag();
		bool newMeasurementRecieved();
		void rosMaskImageBrickCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg);
		void rosMaskImagePatchCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg);
		void rosMaskFootprintCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg);
		void rosNContoursCallback(const std_msgs::Int32::Ptr& ros_msg);
		void rosNPatchesCallback(const std_msgs::Int32::Ptr& ros_msg);
		void currentBrickColorCallback(const std_msgs::String& ros_msgs);

		void smStateCallback(const std_msgs::String& ros_msg);


		void publishPointCloud(
			pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, string camera_frame);
		
		void publishDistance(double distance, string which = "brick"); 
		void publishBaseDistance(double distance, string which = "brick"); 
		void publishBaseBiggestZ(double z, string which = "brick");
		void publishClosestPointZ(double z, string which = "brick");

		pcl::PointCloud<pcl::PointXYZ>::Ptr getOrganizedCloudPtr();
		// vector< vector <int>> getMask();
		void getMask(vector < vector<int>> &mask_brick_loc, vector < vector<int>> &mask_patch_loc);

	private:
		ros::NodeHandle nodeHandle_;
		ros::Subscriber sub_pc2_;
		ros::Subscriber sub_mask_brick_;
		ros::Subscriber sub_mask_patch_;
		ros::Subscriber sub_nContours_;
		ros::Subscriber sub_nPatches_;
		ros::Subscriber sub_current_brick_color_;
		ros::Subscriber sub_sm_state_;

		ros::Publisher pub_pc2_;
		ros::Publisher pub_distance_;
		ros::Publisher pub_distance_patch_;
		ros::Publisher pub_base_distance_;
		ros::Publisher pub_base_distance_patch_;
		ros::Publisher pub_base_z_max_;
		ros::Publisher pub_base_z_max_patch_;
		ros::Publisher pub_closest_point_z_;
		ros::Publisher pub_closest_point_z_patch_;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr organizedCloudPtr;
		vector< vector <int>> mask_brick;
		vector< vector <int>> mask_patch;
		vector< vector <int>> mask_footprint;
		double closest_point_distance;
		bool _newMeasurement = false;
		string current_brick_color_;
};

#endif /* PC_PUB_SUB_H_ */
