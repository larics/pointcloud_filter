/*
 * brick_detection.cpp
 *
 *  Created on: Mar 21, 2019
 *      Author: a
 */

#include "pointcloud_filter.h"
#include "KalmanDetection.h"

void PointcloudFilter::filter ( int argc, char** argv, 
								string pointcloud_sub_topic, 
								string mask_sub_topic_brick, 
								string mask_sub_topic_patch, 
								string mask_sub_topic_footprint,
								string filtered_pointcloud_pub_topic, 
								string closest_point_distance_pub_topic,
								string closest_point_base_distance_pub_topic,
								string camera_frame ) 
{
	ros::init(argc, argv, "pc_filter");
	ros::NodeHandle nodeHandle("~");

	PC_PUB_SUB pcl_pub_sub(	nodeHandle, pointcloud_sub_topic, filtered_pointcloud_pub_topic, 
							closest_point_distance_pub_topic, closest_point_base_distance_pub_topic, 
							mask_sub_topic_brick, mask_sub_topic_patch, mask_sub_topic_footprint);
	const double rate = 50;
	const double dt = 1.0 / 9.0;
	ros::Rate loop_rate(rate);

	tf::TransformListener tf_listener;

	ros::Duration(3.0).sleep();
	KalmanDetection closestDistKF( ros::this_node::getNamespace() +  "/brick_filter/camera_closest_distance");
	KalmanDetection baseDistKF( ros::this_node::getNamespace() + "/brick_filter/base_closest_x");
	KalmanDetection baseZKF( ros::this_node::getNamespace() + "/brick_filter/base_max_z");

	closestDistKF.initializeParameters(nodeHandle);
	baseDistKF.initializeParameters(nodeHandle);
	baseZKF.initializeParameters(nodeHandle);

	KalmanDetection closestDistKFpatch( ros::this_node::getNamespace() + "/patch_filter/camera_closest_distance");
	KalmanDetection baseDistKFpatch( ros::this_node::getNamespace() + "/patch_filter/base_closest_x");
	KalmanDetection baseZKFpatch( ros::this_node::getNamespace() + "/patch_filter/base_max_z");

	closestDistKFpatch.initializeParameters(nodeHandle);
	baseDistKFpatch.initializeParameters(nodeHandle);
	baseZKFpatch.initializeParameters(nodeHandle);

	while(nodeHandle.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		
		bool newMeas = pcl_pub_sub.newMeasurementRecieved();
		if(!newMeas) continue;

		pcXYZ::Ptr originalCloud = pcl_pub_sub.getOrganizedCloudPtr();
		if(!originalCloud || originalCloud->points.size() == 0) continue;

		pcXYZ::Ptr filteredCloud ( new pcXYZ );

		vector < vector<int>> mask_brick, mask_patch, mask_footprint;
		pcl_pub_sub.getMask(mask_brick, mask_patch, mask_footprint);
		
		if (pcl_pub_sub.calc_brick_ || (pcl_pub_sub.calc_patch_ && (pcl_pub_sub.nPatches == 0))) {
			filteredCloud = removeNonMaskValues(originalCloud, mask_brick);
			filteredCloud = removeNaNValues(filteredCloud);

			if (pcl_pub_sub.nContours == 0) {
				double closest_point_distance = -1.0;
				double closest_point_z = -1.0;
				closestDistKF.filterCurrentDistance(dt, closest_point_distance, newMeas);
				pcl_pub_sub.publishDistance( closest_point_distance );
				pcl_pub_sub.publishClosestPointZ( closest_point_z );
			}
			else {
				double closest_point_distance, closest_point_z;
				findClosestDistanceAndClosestPointZ(filteredCloud, closest_point_distance, closest_point_z);
				closestDistKF.filterCurrentDistance(dt, closest_point_distance, newMeas);
				pcl_pub_sub.publishDistance( closest_point_distance );
				pcl_pub_sub.publishClosestPointZ( closest_point_z );
			}
			closestDistKF.publish();

			pcXYZ::Ptr transformedFilteredCloud ( new pcXYZ );
			string goal_frame = "base_footprint";

			tf::StampedTransform temp_trans;

			try 
			{
				tf_listener.lookupTransform( 	goal_frame, filteredCloud->header.frame_id, 
												ros::Time(0), temp_trans );
				pcl_ros::transformPointCloud(*filteredCloud, *transformedFilteredCloud, temp_trans);
				double closest_x_base, biggest_z_base;
				findClosestXAndBiggestZ(transformedFilteredCloud, closest_x_base, biggest_z_base); 
				baseDistKF.filterCurrentDistance(dt, closest_x_base, newMeas);
				baseDistKF.publish();
				pcl_pub_sub.publishBaseDistance(closest_x_base);
				baseZKF.filterCurrentDistance(dt, biggest_z_base, newMeas);
				baseZKF.publish();
				pcl_pub_sub.publishBaseBiggestZ(biggest_z_base);
			}
			catch ( tf::TransformException ex ){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}

			pcl_pub_sub.resetNewMeasurementFlag();

		}

		if (pcl_pub_sub.calc_footprint_) {
			cout << "calc_footprint!" << endl << endl;
			
			filteredCloud = removeNonMaskValues(originalCloud, mask_footprint);

			if(!filteredCloud || filteredCloud->points.size() == 0) cout <<"point size " << filteredCloud->points.size() << endl << endl;

			filteredCloud = removeNaNValues(filteredCloud);
			try 
			{
				string goal_frame = "base_footprint";
				tf::StampedTransform temp_trans;
				auto transformedFilteredCloud = boost::make_shared<pcXYZ>();
				tf_listener.lookupTransform( 	goal_frame, filteredCloud->header.frame_id, 
												ros::Time(0), temp_trans );
				pcl_ros::transformPointCloud(*filteredCloud, *transformedFilteredCloud, temp_trans);
				double closest_x_base, biggest_z_base;
				findClosestXAndBiggestZ(transformedFilteredCloud, closest_x_base, biggest_z_base);
				pcl_pub_sub.publishBaseDistance(closest_x_base, "footprint");
			}
			catch ( tf::TransformException ex ){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}

			pcl_pub_sub.resetNewMeasurementFlag();
		}

		if ((pcl_pub_sub.calc_patch_) && (pcl_pub_sub.nPatches > 0)) {
			filteredCloud = removeNonMaskValues(originalCloud, mask_patch);
			filteredCloud = removeNaNValues(filteredCloud);

			if (pcl_pub_sub.nPatches == 0) {
				double closest_point_distance = -1.0;
				double closest_point_z = -1.0;
				closestDistKFpatch.filterCurrentDistance(dt, closest_point_distance, newMeas);
				pcl_pub_sub.publishDistance( closest_point_distance , "patch");
				pcl_pub_sub.publishClosestPointZ( closest_point_z, "patch" );
			}
			else {
				double closest_point_distance, closest_point_z;
				findClosestDistanceAndClosestPointZ(filteredCloud, closest_point_distance, closest_point_z);
				closestDistKFpatch.filterCurrentDistance(dt, closest_point_distance, newMeas);
				pcl_pub_sub.publishDistance( closest_point_distance, "patch");
				pcl_pub_sub.publishClosestPointZ( closest_point_z, "patch");
			}
			closestDistKFpatch.publish();

			pcXYZ::Ptr transformedFilteredCloud ( new pcXYZ );
			string goal_frame = "base_footprint";

			tf::StampedTransform temp_trans;

			try 
			{
				tf_listener.lookupTransform(goal_frame, filteredCloud->header.frame_id, 
												ros::Time(0), temp_trans );
				pcl_ros::transformPointCloud(*filteredCloud, *transformedFilteredCloud, temp_trans);
				double closest_x_base, biggest_z_base;
				findClosestXAndBiggestZ(transformedFilteredCloud, closest_x_base, biggest_z_base); 
				baseDistKFpatch.filterCurrentDistance(dt, closest_x_base, newMeas);
				baseDistKFpatch.publish();
				pcl_pub_sub.publishBaseDistance(closest_x_base, "patch");
				baseZKFpatch.filterCurrentDistance(dt, biggest_z_base, newMeas);
				baseZKFpatch.publish();
				pcl_pub_sub.publishBaseBiggestZ(biggest_z_base, "patch");
			}
			catch ( tf::TransformException ex ){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}

			pcl_pub_sub.resetNewMeasurementFlag();
		}
		else {
			double not_interested = -1.0;

			closestDistKF.filterCurrentDistance(dt, not_interested, newMeas);
			pcl_pub_sub.publishDistance( not_interested );
			pcl_pub_sub.publishClosestPointZ( not_interested );

			baseDistKF.filterCurrentDistance(dt, not_interested, newMeas);
			baseDistKF.publish();
			pcl_pub_sub.publishBaseDistance(not_interested);
			baseZKF.filterCurrentDistance(dt, not_interested, newMeas);
			baseZKF.publish();
			pcl_pub_sub.publishBaseBiggestZ(not_interested);

			closestDistKFpatch.filterCurrentDistance(dt, not_interested, newMeas);
			pcl_pub_sub.publishDistance( not_interested , "patch");
			pcl_pub_sub.publishClosestPointZ( not_interested, "patch" );

			baseDistKFpatch.filterCurrentDistance(dt, not_interested, newMeas);
			baseDistKFpatch.publish();
			pcl_pub_sub.publishBaseDistance(not_interested, "patch");
			baseZKFpatch.filterCurrentDistance(dt, not_interested, newMeas);
			baseZKFpatch.publish();
			pcl_pub_sub.publishBaseBiggestZ(not_interested, "patch");
		}


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
	if (min_x == inf_x) {
		return -1;
	}
	return min_x;
}

void PointcloudFilter::findClosestXAndBiggestZ(pcXYZ::Ptr inputCloud, double &min_x, double &max_z)
{
	double inf = 99999.9;
	min_x = inf;
	max_z = -1.0;
	if(!inputCloud || inputCloud->points.size() == 0) {
		min_x = -1.;
		max_z = -1.;
	}
	else {
		for (int i = 0; i < inputCloud->points.size(); i++) {
			double x_i = inputCloud->points[i].x;
			double z_i = inputCloud->points[i].z;
			if (x_i < min_x) {
				min_x = x_i;
			}
			if (z_i > max_z) {
				max_z = z_i;
			}
		}
	}
	if (min_x == inf) {
		min_x = -1;
	}
}

void PointcloudFilter::findClosestDistanceAndClosestPointZ(pcXYZ::Ptr inputCloud, double &closest_point_distance, double &closest_point_z)
{
	double inf_distance = 99999.9;
	double min_distance = inf_distance;
	double min_x = 0.0;
	double min_y = 0.0;
	double min_z = 0.0;

	if(!inputCloud || inputCloud->points.size() == 0){
		closest_point_distance = -1;
		closest_point_z = -1;
	}

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

	closest_point_distance = min_distance; 
	closest_point_z = min_z;

	if (min_distance == inf_distance) {
		closest_point_distance = -1;
		closest_point_z = -1;
	}
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
	if (min_distance == inf_distance)
		return -1.0;
		
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



