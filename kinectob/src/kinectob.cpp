#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>   
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/calib3d/calib3d.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
		cv::waitKey(1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::imshow("depth", cv_bridge::toCvShare(msg, "16UC1")->image);
		cv::waitKey(1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
	}
}

// kinect2/hd/image_color_rect  (1920,1080)  
// kinect2/qhd/image_color_rect (960,540)
// kinect2/sd/image_color_rect  (512,424)

int main(int argc, char** argv)
{
	//init
    	ros::init(argc, argv, "kinectob");
    	ros::NodeHandle nh;

	cv::namedWindow("view", 0);
	cv::startWindowThread();
	cv::namedWindow("depth", 0);
	cv::startWindowThread();

	//registration
	image_transport::ImageTransport it_color(nh);
	image_transport::ImageTransport it_depth(nh);
	image_transport::Subscriber sub_color = it_color.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
	image_transport::Subscriber sub_depth = it_depth.subscribe("/kinect2/qhd/image_depth_rect", 1, depthCallback);

	//loop
	ros::spin();
	ros::shutdown();

	cv::destroyWindow("view");
	cv::destroyWindow("depth");

	return 0;
}









