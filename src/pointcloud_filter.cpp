/*
 * brick_detection.cpp
 *
 *  Created on: Mar 21, 2019
 *      Author: a
 */

#include "pointcloud_filter.h"
#include "KalmanDetection.h"
#include <pcl/filters/statistical_outlier_removal.h>

#define NO_CONOUTS_ERROR -1

void PointcloudFilter::filter ( int argc, char** argv, 
								string pointcloud_sub_topic, 
								string mask_sub_topic,
								string patch_mask_sub_topic,
								string filtered_pointcloud_pub_topic, 
								string closest_point_distance_pub_topic,
								string closest_point_base_distance_pub_topic,
								string patch_centroid_pub_topic,
								string patch_centroid_filtered_pub_topic,
								string camera_frame ) 
{
	ros::init(argc, argv, "pc_filter");
	ros::NodeHandle nodeHandle("~");

	PC_PUB_SUB pcl_pub_sub(	nodeHandle, pointcloud_sub_topic, mask_sub_topic, patch_mask_sub_topic,
							filtered_pointcloud_pub_topic, closest_point_distance_pub_topic,
							closest_point_base_distance_pub_topic, patch_centroid_pub_topic, 
							patch_centroid_filtered_pub_topic);
	ros::Rate loop_rate(50);
	tf::TransformListener tf_listener;
	ros::Duration(3.0).sleep();

	KalmanDetection KFabsBrickDistance("brick");
	KalmanDetection KFpatchCentroidX("centroid_x");
	KalmanDetection KFpatchCentroidY("centroid_y");
	KalmanDetection KFpatchCentroidZ("centroid_z");

	double dt = 1.0 / 50.0;
	KFabsBrickDistance.initializeParameters(nodeHandle);
	KFpatchCentroidX.initializeParameters(nodeHandle);
	KFpatchCentroidY.initializeParameters(nodeHandle);
	KFpatchCentroidZ.initializeParameters(nodeHandle);

	while(nodeHandle.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		
		pcXYZ::Ptr originalCloud = pcl_pub_sub.getOrganizedCloudPtr();

		if(!originalCloud || originalCloud->points.size() == 0) {
			ROS_WARN("no cloud!");
			continue;
		}

		pcXYZ::Ptr filteredCloud ( new pcXYZ ), patchCloud( new pcXYZ );

		// filteredCloud = removeNonMaskValues(originalCloud, pcl_pub_sub.getMask());
		// filteredCloud = removeNaNValues(filteredCloud);
		// filteredCloud = doOutlierFiltering(filteredCloud, nodeHandle);

		patchCloud = removeNonMaskValues(originalCloud, pcl_pub_sub.getMask());
		if (!patchCloud->empty()) {
			patchCloud = doOutlierFiltering(patchCloud, nodeHandle);
		}
		patchCloud = removeNaNValues(patchCloud);
		if (!patchCloud->empty()) {
			patchCloud = doOutlierFiltering(patchCloud, nodeHandle);
		}

		//pcl_pub_sub.publishPointCloud(filteredCloud, camera_frame);
		std::vector<double> minDistances, patchCentroid;
		if (pcl_pub_sub.nContours == 0) {
			//cout << "trazi closest od org clouda!" << endl << endl;
			minDistances = std::vector<double>(4, NO_CONOUTS_ERROR);
			patchCentroid = std::vector<double>(3, NO_CONOUTS_ERROR);
		}
		else {
		    minDistances = findClosestDistance(filteredCloud);
		    patchCentroid = findCentroid(patchCloud);
		}
		// Publish the absolute distance
		pcl_pub_sub.publishDistance(minDistances[3]);
		pcl_pub_sub.publishPatchCentroidVector(patchCentroid);

		bool newMeas = pcl_pub_sub.newMeasurementRecieved();
		// Do the Kalman Filter Here
		// KFabsBrickDistance.filterCurrentDistance(dt, minDistances[3], newMeas);
		KFpatchCentroidX.filterCurrentDistance(dt, patchCentroid[0], newMeas);
		KFpatchCentroidY.filterCurrentDistance(dt, patchCentroid[1], newMeas);
		KFpatchCentroidZ.filterCurrentDistance(dt, patchCentroid[2], newMeas);
		// KFabsBrickDistance.publish();
		pcl_pub_sub.resetNewMeasurementFlag();

		pcl_pub_sub.publishPatchCentroidFilteredVector(
			std::vector<double> {
				KFpatchCentroidX.getState(),
				KFpatchCentroidY.getState(),
				KFpatchCentroidZ.getState()
			}
		);
		pcXYZ::Ptr transformedFilteredCloud ( new pcXYZ );
		string goal_frame = "base_link";

		tf::StampedTransform temp_trans;

		pcl_pub_sub.publishPointCloud(patchCloud, patchCloud->header.frame_id);
		pcl_pub_sub.publishBaseDistance( minDistances[0] );
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


pcXYZ::Ptr PointcloudFilter::doOutlierFiltering( pcXYZ::Ptr inputCloud , ros::NodeHandle& nh)
{
	double meanK = 50, stddevMulThres = 1;
	nh.getParam("brick/outlier/mean_k", meanK);
	nh.getParam("brick/outlier/stddev_multiplier_thresh", stddevMulThres);

	pcXYZ::Ptr cloud_filtered {new pcXYZ};
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (inputCloud);
	sor.setMeanK (meanK);
	sor.setStddevMulThresh (stddevMulThres);
	sor.filter (*cloud_filtered);

	return cloud_filtered;
}

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

std::vector<double> PointcloudFilter::findClosestDistance(pcXYZ::Ptr inputCloud)
{
	double inf_distance = 99999.9;
	double min_distance = inf_distance;
	double min_x = 0.0;
	double min_y = 0.0;
	double min_z = 0.0;

	if(!inputCloud || inputCloud->points.size() == 0)	return std::vector<double> {-1, -1, -1, -1};
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
	return std::vector<double> {min_x, min_y, min_z, min_distance};
}

std::vector<double> PointcloudFilter::findCentroid(pcXYZ::Ptr inputCloud)
{
    std::vector<double> centroid{0,0,0};
    if (!inputCloud || inputCloud->points.size() == 0) return std::vector<double>{-1, -1, -1};
    else {
        for (int i = 0; i < inputCloud->points.size(); i++) {
            centroid[0] += inputCloud->points[i].x;
            centroid[1] += inputCloud->points[i].y;
            centroid[2] += inputCloud->points[i].z;
        }
        centroid[0] /= inputCloud->points.size();
        centroid[1] /= inputCloud->points.size();
        centroid[2] /= inputCloud->points.size();
    }
    return centroid;
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



