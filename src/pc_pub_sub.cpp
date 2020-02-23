#include "pc_pub_sub.h"

PC_PUB_SUB::PC_PUB_SUB(	ros::NodeHandle& nodeHandle, string pointcloud_sub_topic, 
						string filtered_pointcloud_pub_topic, 
						string closest_point_distance_pub_topic,
						string closest_point_base_distance_pub_topic,
						string mask_sub_topic_brick,
						string mask_sub_topic_patch) 
{
	nodeHandle_ = nodeHandle;
	registerPointCloudSubscriber(pointcloud_sub_topic);
	string topic = "/erl_husky/red/red_color_filter/nContours";
	registerNContoursSubscriber(topic);
	topic = "/erl_husky/red/red_color_filter/nPatches";
	registerNPatchesSubscriber(topic);
	if (mask_sub_topic_brick != "none")
		registerImageSubscriberBrick(mask_sub_topic_brick);
	if (mask_sub_topic_patch != "none")
		registerImageSubscriberPatch(mask_sub_topic_patch);
	registerPointCloudPublisher(filtered_pointcloud_pub_topic);

	registerDistancePublisher(closest_point_distance_pub_topic, closest_point_distance_pub_topic + "_patch");
	registerBaseDistancePublisher(closest_point_base_distance_pub_topic, closest_point_base_distance_pub_topic + "_patch");
	topic = "pc_filter/base_z_max";
	registerBaseMaxZPublisher(topic, topic + "_patch");
	topic = "pc_filter/closest_point_z";
	registerClosestPointZPublisher(topic, topic + "_patch");

	topic = "/brick_color";
	registerBrickColorSubscriber(topic);

	topic = "/sm_state";
	registerSmStateSubscriber(topic);

	current_brick_color_ = "r";
}

PC_PUB_SUB::~PC_PUB_SUB() 
{
}
 
void PC_PUB_SUB::resetNewMeasurementFlag()
{
       _newMeasurement = false;
}

bool PC_PUB_SUB::newMeasurementRecieved()
{
       return _newMeasurement;
}

void PC_PUB_SUB::registerPointCloudSubscriber(string topic) 
{
	sub_pc2_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::rosPointCloudCallback, this);
}
void PC_PUB_SUB::registerImageSubscriberBrick(string topic) 
{
	sub_mask_brick_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::rosMaskImageBrickCallback, this);
}

void PC_PUB_SUB::registerImageSubscriberPatch(string topic) 
{
	sub_mask_patch_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::rosMaskImagePatchCallback, this);
}

void PC_PUB_SUB::registerNContoursSubscriber(string topic)
{
	sub_nContours_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::rosNContoursCallback, this);
}

void PC_PUB_SUB::registerNPatchesSubscriber(string topic)
{
	sub_nPatches_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::rosNPatchesCallback, this);
}


void PC_PUB_SUB::registerBrickColorSubscriber(string topic) {

	sub_current_brick_color_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::currentBrickColorCallback, this);

}

void PC_PUB_SUB::registerSmStateSubscriber(string topic) {

	sub_sm_state_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::smStateCallback, this);

}


void PC_PUB_SUB::registerPointCloudPublisher(string topic) 
{
	pub_pc2_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(topic, 1);
}
void PC_PUB_SUB::registerDistancePublisher(string topic, string topic_patch)
{
	pub_distance_ = nodeHandle_.advertise<std_msgs::Float32>(topic, 1);
	pub_distance_patch_ = nodeHandle_.advertise<std_msgs::Float32>(topic_patch, 1);
}
void PC_PUB_SUB::registerBaseDistancePublisher(string topic, string topic_patch)
{
	pub_base_distance_ = nodeHandle_.advertise<std_msgs::Float32>(topic, 1);
	pub_base_distance_patch_ = nodeHandle_.advertise<std_msgs::Float32>(topic_patch, 1);
}
void PC_PUB_SUB::registerBaseMaxZPublisher(string topic, string topic_patch)
{
	pub_base_z_max_ = nodeHandle_.advertise<std_msgs::Float32>(topic, 1);
	pub_base_z_max_patch_ = nodeHandle_.advertise<std_msgs::Float32>(topic_patch, 1);
}

void PC_PUB_SUB::registerClosestPointZPublisher(string topic, string topic_patch)
{
	pub_closest_point_z_ = nodeHandle_.advertise<std_msgs::Float32>(topic, 1);
	pub_closest_point_z_patch_ = nodeHandle_.advertise<std_msgs::Float32>(topic_patch, 1);
}

void PC_PUB_SUB::rosPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_msg) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

	vector<int> indices;

	pcl::fromROSMsg(*ros_msg, *pcl_msg);

	this->organizedCloudPtr = pcl_msg;
	_newMeasurement = true;
}

void PC_PUB_SUB::rosNContoursCallback(const std_msgs::Int32::Ptr& ros_msg) 
{
	this->nContours = ros_msg->data;
}

void PC_PUB_SUB::rosNPatchesCallback(const std_msgs::Int32::Ptr& ros_msg) 
{
	this->nPatches = ros_msg->data;
}

vector <vector <int>> PC_PUB_SUB::processMaskImage(const cv::Mat image) {
	int width = image.cols;
	int height = image.rows;
	int _stride = image.step;

	uint8_t *myData = image.data;
	vector <vector <int>> image_mat;
	int sum = 0;
	for (int i = 0; i < height; i++) {
		vector <int> temp_vec;
		for (int j = 0; j < width; j++) {
			int d = *(image.data + i * width +j);

			if (d > 255 || d < 0) {
				sum++;
			}
			temp_vec.push_back( *(image.data + i * width +j));
		}
		image_mat.push_back(temp_vec);
	}
	return image_mat;
}

void PC_PUB_SUB::rosMaskImageBrickCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg) 
{
	cv::Mat image = cv::imdecode(cv::Mat(ros_msg->data), -1);

	mask_brick = PC_PUB_SUB::processMaskImage(image);
}

void PC_PUB_SUB::rosMaskImagePatchCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg) 
{
	cv::Mat image = cv::imdecode(cv::Mat(ros_msg->data), -1);

	mask_patch = PC_PUB_SUB::processMaskImage(image);

}

void PC_PUB_SUB::rosMaskFootprintCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg)
{
	const auto image = cv::imdecode(cv::Mat(ros_msg->data), -1);
	mask_footprint = PC_PUB_SUB::processMaskImage(image);
}

void PC_PUB_SUB::smStateCallback(const std_msgs::String& ros_msg) 
{
  std::string sm_state = ros_msg.data;

  if (sm_state == "husky_servo_color") {
	calc_brick_ = true;
	calc_patch_ = true;
  }
  else if ((sm_state == "husky_servo_patch") || (sm_state == "schunk_repeat_servo") || (sm_state == "schunk_servo_pickup")) {
	  calc_brick_ = false;
	  calc_patch_ = true;
  }
  else {
	  calc_brick_ = false;
	  calc_patch_ = false;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PC_PUB_SUB::getOrganizedCloudPtr() 
{
	return organizedCloudPtr;
}

void PC_PUB_SUB::getMask(vector < vector<int>> &mask_brick_loc, vector < vector<int>> &mask_patch_loc) {
	mask_brick_loc = mask_brick;
	mask_patch_loc = mask_patch;
}

void PC_PUB_SUB::publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, string camera_frame) 
{
	sensor_msgs::PointCloud2::Ptr ros_msg(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*pointCloud, *ros_msg);

	std_msgs::Header head;
	head.stamp = ros::Time::now();
	//head.frame_id = "camera_link";
	//head.frame_id = "camera_color_optical_frame";
	head.frame_id = camera_frame;
	ros_msg->header = head;

	pub_pc2_.publish(*ros_msg);
}

void PC_PUB_SUB::publishDistance(double distance, string which) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = distance;
	if (which == "brick")
		pub_distance_.publish(*msg);
	else if (which == "patch")
		pub_distance_patch_.publish(*msg);
	
}

void PC_PUB_SUB::publishBaseDistance(double distance, string which) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = distance;
	if (which == "brick")
		pub_base_distance_.publish(*msg);
	else if (which == "patch")
		pub_base_distance_patch_.publish(*msg);
}

void PC_PUB_SUB::publishBaseBiggestZ(double z, string which) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = z;
	if (which == "brick")
		pub_base_z_max_.publish(*msg);
	else if (which == "patch")
		pub_base_z_max_patch_.publish(*msg);
}

void PC_PUB_SUB::publishClosestPointZ(double z, string which) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = z;
	if (which == "brick")
		pub_closest_point_z_.publish(*msg);
	else if (which == "patch")
		pub_closest_point_z_patch_.publish(*msg);
}

void PC_PUB_SUB::currentBrickColorCallback(const std_msgs::String& ros_msgs )
{
  std::string new_brick_color = ros_msgs.data;
  if (new_brick_color != current_brick_color_) {

    string new_color, old_color;
    if (current_brick_color_ == "r") {
      old_color = "red";
    }
    else if (current_brick_color_ == "g") {
      old_color = "green";
    }
    else if (current_brick_color_ == "b") {
      old_color = "blue";
    }

    current_brick_color_ = ros_msgs.data;

    if (current_brick_color_ == "r") {
      new_color = "red";
    }
    else if (current_brick_color_ == "g") {
      new_color = "green";
    }
    else if (current_brick_color_ == "b") {
      new_color = "blue";
    }

	cout << "old color = " << old_color << ", new color = " << new_color << endl;
	string old_header = "/erl_husky/" + old_color + "/" + old_color;  
	string new_header = "/erl_husky/" + new_color + "/" + new_color;  


	string current_topic = sub_mask_brick_.getTopic();
	cout << "old_topic1 = " << current_topic << endl << endl;
	current_topic.replace(current_topic.find(old_header), old_header.length(), new_header);
	sub_mask_brick_.shutdown();
	sub_mask_brick_ = nodeHandle_.subscribe(current_topic, 1, &PC_PUB_SUB::rosMaskImageBrickCallback, this);
	cout << "new_topic = " << current_topic << endl << endl;

	current_topic = sub_nContours_.getTopic();
	current_topic.replace(current_topic.find(old_header), old_header.length(), new_header);
	sub_nContours_.shutdown();
	sub_nContours_ = nodeHandle_.subscribe(current_topic, 1, &PC_PUB_SUB::rosNContoursCallback, this);

	current_topic = sub_mask_patch_.getTopic();
	current_topic.replace(current_topic.find(old_header), old_header.length(), new_header);
	sub_mask_patch_.shutdown();
	sub_mask_patch_ = nodeHandle_.subscribe(current_topic, 1, &PC_PUB_SUB::rosMaskImagePatchCallback, this);

	current_topic = sub_nPatches_.getTopic();
	current_topic.replace(current_topic.find(old_header), old_header.length(), new_header);
	sub_nPatches_.shutdown();
	sub_nPatches_ = nodeHandle_.subscribe(current_topic, 1, &PC_PUB_SUB::rosNPatchesCallback, this);


  }
  std::cout << "curr brick color = " << current_brick_color_ << std::endl;
}