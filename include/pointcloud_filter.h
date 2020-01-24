/*
 * brick_detection.h
 *
 *  Created on: Mar 21, 2019
 *      Author: a
 */

#ifndef INCLUDE_BRICK_DETECTION_H_
#define INCLUDE_BRICK_DETECTION_H_

#include <Eigen/Dense>
#include <math.h>
#include <ctime>
#include <vector>

#include "pc_pub_sub.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pcXYZ;

namespace PointcloudFilter {

	void filter ( int argc, char** argv, 
				string pointcloud_sub_topic, 
				string mask_sub_topic, 
				string filtered_pointcloud_pub_topic, 
				string closest_point_distance_pub_topic,
				string closest_point_base_distance_pub_topic,
				string camera_frame );

	// removeNonMaskValues 
	// returns pointloud containing 'inputCloud' points corresponding 
	// to white pixels of 'mask'
	pcXYZ::Ptr removeNonMaskValues( pcXYZ::Ptr inputCloud, vector <vector <int>> mask );

	// returns unorganized pointcloud without NaN values
	pcXYZ::Ptr removeNaNValues ( pcXYZ::Ptr inputCloud );

	//pcXYZ::Ptr transformCloud(pcXYZ::Ptr inputCloud, string goal_frame, tf::TransformListener tf_listener);

	vector <Eigen::Vector3d> pointIndicesToInlierPoints ( pcXYZ::Ptr inputCloud, 
		pcl::PointIndices::Ptr inliers);


	double findClosestDistance(pcXYZ::Ptr inputCloud);
	void findClosestDistanceAndClosestPointZ(pcXYZ::Ptr inputCloud, double &closest_point_distance, double &closest_point_z);
	double findClosestX(pcXYZ::Ptr inputCloud);
	void findClosestXAndBiggestZ(pcXYZ::Ptr inputCloud, double &min_x, double &max_z);


};




#endif /* INCLUDE_BRICK_DETECTION_H_ */
