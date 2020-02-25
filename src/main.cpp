#include <iostream>
#include "pointcloud_filter.h"

using namespace std;

int main(int argc, char** argv) {

	// ------------ realsense ----------------
	string pointcloud_sub_topic = "/erl_husky/camera/depth_registered/points";
	//string mask_sub_topic = "/erl_husky/red/red_color_filter/current_patch/compressed";
	string mask_sub_topic_brick = "/brick_mask/compressed";
	string mask_sub_topic_patch = "/patch_mask/compressed";
	string mask_sub_topic_footprint = "/erl_husky/wall_detector/mask/compressed";
	string filtered_pointcloud_pub_topic = "pc_filter/points";
	string closest_point_distance_pub_topic = "pc_filter/closest_point_distance";
	string closest_point_base_distance_pub_topic = "pc_filter/base_closest_x";
	string camera_frame = "camera_link";

	PointcloudFilter::filter ( 	argc, argv, pointcloud_sub_topic, 
								mask_sub_topic_brick,
								mask_sub_topic_patch,
								mask_sub_topic_footprint,
								filtered_pointcloud_pub_topic, 
								closest_point_distance_pub_topic, 
								closest_point_base_distance_pub_topic, 
								camera_frame);
}
