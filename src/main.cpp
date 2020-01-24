#include <iostream>
#include "pointcloud_filter.h"

using namespace std;

int main(int argc, char** argv) {

	// ------------ realsense ----------------
	string pointcloud_sub_topic = "/erl_husky/camera/depth_registered/points";
	string mask_sub_topic = "/erl_husky/red/red_color_filter/current_patch/compressed";
	string filtered_pointcloud_pub_topic = "/pc_filter/points";
	string closest_point_distance_pub_topic = "/pc_filter/closest_point_distance";
	string closest_point_base_distance_pub_topic = "/pc_filter/base_closest_point_distance";
	string camera_frame = "camera_link";

	PointcloudFilter::filter ( 	argc, argv, pointcloud_sub_topic, mask_sub_topic,
								filtered_pointcloud_pub_topic, 
								closest_point_distance_pub_topic, 
								closest_point_base_distance_pub_topic, 
								camera_frame);

}
