/*
 * brick_detection.cpp
 *
 *  Created on: Mar 21, 2019
 *      Author: a
 */

#include "pointcloud_filter.h"

void PointcloudFilter::filter ( int argc, char** argv, 
								string pointcloud_sub_topic, 
								string mask_sub_topic, 
								string filtered_pointcloud_pub_topic, 
								string closest_point_distance_pub_topic,
								string closest_point_base_distance_pub_topic,
								string camera_frame ) 
{
	ros::init(argc, argv, "pc_filter");
	ros::NodeHandle nodeHandle("~");

	PC_PUB_SUB pcl_pub_sub(	nodeHandle, pointcloud_sub_topic, filtered_pointcloud_pub_topic, 
							closest_point_distance_pub_topic, closest_point_base_distance_pub_topic, 
							mask_sub_topic);
	ros::Rate loop_rate(50);

	tf::TransformListener tf_listener;

	ros::Duration(3.0).sleep();
	
	while(nodeHandle.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		
		pcXYZ::Ptr originalCloud = pcl_pub_sub.getOrganizedCloudPtr();

		if(!originalCloud || originalCloud->points.size() == 0) continue;

		pcXYZ::Ptr filteredCloud ( new pcXYZ );

		filteredCloud = removeNonMaskValues(originalCloud, pcl_pub_sub.getMask());
		filteredCloud = removeNaNValues(filteredCloud);

		//pcl_pub_sub.publishPointCloud(filteredCloud, camera_frame);
		if (pcl_pub_sub.nContours == 0) {
			//cout << "trazi closest od org clouda!" << endl << endl;
			pcl_pub_sub.publishDistance( findClosestDistance(originalCloud) );
		}
		else {
			//cout << "trazi closest od filtriranog clouda!" << endl << endl;
			pcl_pub_sub.publishDistance( findClosestDistance(filteredCloud) );
		}

		pcXYZ::Ptr transformedFilteredCloud ( new pcXYZ );
		string goal_frame = "base_link";

		tf::StampedTransform temp_trans;

		pcl_pub_sub.publishPointCloud(filteredCloud, filteredCloud->header.frame_id);
		//pcl_pub_sub.publishPointCloud(filteredCloud, goal_frame);
		//pcl_pub_sub.publishDistance( findClosestDistance(filteredCloud) );
		pcl_pub_sub.publishBaseDistance( findClosestX(transformedFilteredCloud) );
	}
}
/*
pcXYZ::Ptr PointcloudFilter::transformCloud(pcXYZ::Ptr inputCloud, string goal_frame, tf::TransformListener tf_listener) 
{
	//cout << "input frame_id = " << inputCloud->header.frame_id << endl << endl;
	pcXYZ::Ptr outputCloud ( new pcXYZ );

	//tf::TransformListener tf_listener(ros::Duration(1));
	tf::StampedTransform temp_trans;
	//cout << "input_frame_id" << inputCloud->header.frame_id << endl << endl;
	//tf_listener.waitForTransform( goal_frame, inputCloud->header.frame_id, ros::Time(0), ros::Duration(1));
	tf_listener.lookupTransform( goal_frame, inputCloud->header.frame_id, ros::Time(0), temp_trans);
	
	pcl_ros::transformPointCloud(*inputCloud, *outputCloud, temp_trans);
	//pcl_ros::transformPointCloud(goal_frame, *inputCloud, *outputCloud, tf_listener);
	return outputCloud;
}
 */
double PointcloudFilter::findClosestX(pcXYZ::Ptr inputCloud)
{
	double inf_x = 99999.9;
	double min_x = inf_x;
	if(!inputCloud || inputCloud->points.size() == 0)	return -1;
	else {
		for (int i = 0; i < inputCloud->points.size(); i++) {
			double x_i = inputCloud->points[i].x;
			if (x_i < min_x) {
				min_x = x_i;
			}
		}
	}
	return min_x;
}

double PointcloudFilter::findClosestDistance(pcXYZ::Ptr inputCloud)
{
	double inf_distance = 99999.9;
	double min_distance = inf_distance;
	double min_x = 0.0;
	double min_y = 0.0;
	double min_z = 0.0;

	if(!inputCloud || inputCloud->points.size() == 0)	return -1;
	else {
		for (int i = 0; i < inputCloud->points.size(); i++) {
			double x = inputCloud->points[i].x;
			double y = inputCloud->points[i].y;
			double z = inputCloud->points[i].z;
			double distance_i = sqrt ( x*x + y*y + z*z );
			if (distance_i < min_distance) {
				min_distance = distance_i;
				min_x = x;
				min_y = y;
				min_z = z;
			}
		}
	}
	//cout << "closest point: (" << min_x << ", " << min_y << ", " << min_z << ")" << endl << endl;
	return min_distance;
}

vector <Eigen::Vector3d> PointcloudFilter::pointIndicesToInlierPoints (
	pcXYZ::Ptr inputCloud, pcl::PointIndices::Ptr inliers )
{
	Eigen::Vector3d tempPoint;
	vector <Eigen::Vector3d> inlierPoints;
	for(int inlierCounter = 0; inlierCounter < inliers->indices.size(); inlierCounter++) {
		tempPoint << 	inputCloud->points[inliers->indices[inlierCounter]].x,
						inputCloud->points[inliers->indices[inlierCounter]].y,
						inputCloud->points[inliers->indices[inlierCounter]].z;
		inlierPoints.push_back ( tempPoint );
	}
	return inlierPoints;
}

pcXYZ::Ptr PointcloudFilter::removeNaNValues ( pcXYZ::Ptr inputCloud )
{
	vector<int> indices;
	pcXYZ::Ptr tempCloud ( new pcXYZ );
	pcl::removeNaNFromPointCloud ( *inputCloud, *tempCloud, indices );
	return tempCloud;
}
pcXYZ::Ptr PointcloudFilter::removeNonMaskValues( pcXYZ::Ptr inputCloud, 
												vector <vector <int>> mask )
{
	pcXYZ::Ptr tempCloud ( new pcXYZ );
	*tempCloud = *inputCloud;

	tempCloud->clear();
	tempCloud->header = inputCloud->header;
	tempCloud->height = 1;
	tempCloud->width = 0;
	for (int i = 0; i < mask.size(); i++) {
		for (int j = 0; j < mask[0].size(); j++) {
			if (mask[i][j] > 200) {
				tempCloud->width++;
				tempCloud->points.push_back(inputCloud->at(j,i));
			}
		}
	}
	return tempCloud;
}



