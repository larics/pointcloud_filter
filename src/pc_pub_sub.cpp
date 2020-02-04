#include "pc_pub_sub.h"

PC_PUB_SUB::PC_PUB_SUB(	ros::NodeHandle& nodeHandle, string pointcloud_sub_topic, 
						string filtered_pointcloud_pub_topic, 
						string closest_point_distance_pub_topic,
						string closest_point_base_distance_pub_topic,
						string mask_sub_topic) 
{
	nodeHandle_ = nodeHandle;
	registerPointCloudSubscriber(pointcloud_sub_topic);
	registerNContoursSubscriber();
	if (mask_sub_topic != "none")
		registerImageSubscriber(mask_sub_topic);
	registerPointCloudPublisher(filtered_pointcloud_pub_topic);
	registerDistancePublisher(closest_point_distance_pub_topic);
	registerBaseDistancePublisher(closest_point_base_distance_pub_topic);
	registerBaseMaxZPublisher();
	registerClosestPointZPublisher();
	sub_current_brick_color_ = nodeHandle_.subscribe("/brick_color", 1, &PC_PUB_SUB::currentBrickColorCallback, this);
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
void PC_PUB_SUB::registerImageSubscriber(string topic) 
{
	sub_mask_ = nodeHandle_.subscribe(topic, 1, &PC_PUB_SUB::rosMaskImageCallback, this);
}

void PC_PUB_SUB::registerNContoursSubscriber()
{
	sub_nContours_ = nodeHandle_.subscribe("/erl_husky/red/red_color_filter/nContours", 1, &PC_PUB_SUB::rosNContoursCallback, this);
}

void PC_PUB_SUB::registerPointCloudPublisher(string topic) 
{
	pub_pc2_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(topic, 1);
}
void PC_PUB_SUB::registerDistancePublisher(string topic)
{
	pub_distance_ = nodeHandle_.advertise<std_msgs::Float32>(topic, 1);
}
void PC_PUB_SUB::registerBaseDistancePublisher(string topic)
{
	pub_base_distance_ = nodeHandle_.advertise<std_msgs::Float32>(topic, 1);
}
void PC_PUB_SUB::registerBaseMaxZPublisher()
{
	pub_base_z_max_ = nodeHandle_.advertise<std_msgs::Float32>("pc_filter/base_z_max", 1);
}

void PC_PUB_SUB::registerClosestPointZPublisher()
{
	pub_closest_point_z_ = nodeHandle_.advertise<std_msgs::Float32>("pc_filter/closest_point_z", 1);
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

void PC_PUB_SUB::rosMaskImageCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_msg) 
{
	cv::Mat image = cv::imdecode(cv::Mat(ros_msg->data), -1);

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

	mask = image_mat;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PC_PUB_SUB::getOrganizedCloudPtr() 
{
	return organizedCloudPtr;
}

vector< vector <int>> PC_PUB_SUB::getMask() {
	return mask;
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

void PC_PUB_SUB::publishDistance(double distance) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = distance;
	pub_distance_.publish(*msg);
}

void PC_PUB_SUB::publishBaseDistance(double distance) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = distance;
	pub_base_distance_.publish(*msg);
}

void PC_PUB_SUB::publishBaseBiggestZ(double z) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = z;
	pub_base_z_max_.publish(*msg);
}

void PC_PUB_SUB::publishClosestPointZ(double z) 
{
	std_msgs::Float32::Ptr msg(new std_msgs::Float32);

	msg->data = z;
	pub_closest_point_z_.publish(*msg);
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
	string current_topic = sub_mask_.getTopic();

	cout << "old_topic1 = " << current_topic << endl << endl;

	string old_header = "/erl_husky/" + old_color + "/" + old_color;  
	string new_header = "/erl_husky/" + new_color + "/" + new_color;  

	current_topic.replace(current_topic.find(old_header), old_header.length(), new_header);
	
	sub_mask_.shutdown();
	sub_mask_ = nodeHandle_.subscribe(current_topic, 1, &PC_PUB_SUB::rosMaskImageCallback, this);
	cout << "new_topic = " << current_topic << endl << endl;

  }
  std::cout << "curr brick color = " << current_brick_color_ << std::endl;
}