#include <rgbd_tools/StereoCamera.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#include <thread>
#include <chrono>


typedef pcl::PointXYZRGBNormal PointType_;

int main (int _argc, char** _argv) {
	// Load arguments
	if (_argc != 2) {
		std::cout << "Bad input argument, provide config file" << std::endl;
		return false;
	}

	std::ifstream file(_argv[1]);
	if (!file.is_open()) {
		std::cout << "Failed to open config file" << std::endl;
		return false;
	}
	cjson::Json configFile;
	if (!configFile.parse(file)) {
		std::cout << "Failed parsing file" << std::endl;
		return false;
	}

	ros::init(_argc, _argv, "ZedPublisher");
	std::cout << "Initialized ros" << std::endl;

	std::thread spinningThread([&](){
		ros::spin();
	});

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher leftStream = it.advertise("/stereo_camera/left/image_rect_color", 1);
	image_transport::Publisher rightStream = it.advertise("/stereo_camera/right/image_rect_color", 1);
	
	ros::Publisher leftInfoStream = nh.advertise<sensor_msgs::CameraInfo>("/stereo_camera/left/camera_info",1);
	ros::Publisher rightInfoStream = nh.advertise<sensor_msgs::CameraInfo>("/stereo_camera/right/camera_info",1);

	// Camera Initialization
	std::cout << "Configuring camera" << std::endl;
	rgbd::StereoCamera *camera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Custom);
	camera->init(configFile);
	std::cout << "Configured camera" << std::endl;	
	
	sensor_msgs::CameraInfo leftInfo, rightInfo;
	leftInfo.height = 720;
	leftInfo.width = 1280;
	leftInfo.distortion_model = "plumb_bob";
	leftInfo.D = {0.0, 0.0, 0.0, 0.0, 0.0};
	leftInfo.K = {674.4103393554688, 0.0, 689.690185546875, 0.0, 674.4103393554688, 374.011962890625, 0.0, 0.0, 1.0};
	leftInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	leftInfo.P = {674.4103393554688, 0.0, 689.690185546875, 0.0, 0.0, 674.4103393554688, 374.011962890625, 0.0, 0.0, 0.0, 1.0, 0.0};
	leftInfo.binning_x = 0;
	leftInfo.binning_y = 0;
	leftInfo.roi.x_offset = 0;
	leftInfo.roi.y_offset = 0;
	leftInfo.roi.width = 0;
	leftInfo.roi.height = 0;
	leftInfo.roi.do_rectify = false;

	rightInfo.height = 720;
	rightInfo.width = 1280;
	rightInfo.distortion_model = "plumb_bob";
	rightInfo.D = {0.0, 0.0, 0.0, 0.0, 0.0};
	rightInfo.K = {674.4103393554688, 0.0, 689.690185546875, 0.0, 674.4103393554688, 374.011962890625, 0.0, 0.0, 1.0};
	rightInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	rightInfo.P = {674.4103393554688, 0.0, 689.690185546875, -80.92924499511719, 0.0, 674.4103393554688, 374.011962890625, 0.0, 0.0, 0.0, 1.0, 0.0};
	rightInfo.binning_x = 0;
	rightInfo.binning_y = 0;
	rightInfo.roi.x_offset = 0;
	rightInfo.roi.y_offset = 0;
	rightInfo.roi.width = 0;
	rightInfo.roi.height = 0;
	rightInfo.roi.do_rectify = false;

	while(ros::ok()){
		ros::Time stamp = ros::Time::now();
		camera->grab();
		cv::Mat left, right;
		camera->rgb(left, right);
		sensor_msgs::ImagePtr leftImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left).toImageMsg();
		leftImage->header.frame_id = "zed_left_camera";
		sensor_msgs::ImagePtr rightImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right).toImageMsg();
		rightImage->header.frame_id = "zed_right_camera";

		leftImage->header.stamp = stamp;
		rightImage->header.stamp = stamp;
		leftInfo.header.stamp = stamp;
		rightInfo.header.stamp = stamp;

		leftStream.publish(leftImage);
		rightStream.publish(rightImage);
		leftInfoStream.publish(leftInfo);
		rightInfoStream.publish(rightInfo);
	}

	return (0);
}
