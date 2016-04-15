#include <ros/ros.h>

#include <stdio.h>
#include <iostream>
#include <depth_to_laser.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  

//subscribe 
Params param;
bool flag = false;

//publisher
ros::Publisher ob_pub;

// camera info subscribe
void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	if (!flag)
	{
		// (f, c_u, c_v, scan_height, translation_x, translation_z)
		param.setParams(info_msg->K[0], info_msg->K[2], info_msg->K[5], 100, -0.2f, -0.2f);
		flag = true;
	}
}

// depth image subscribe
void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
	if (flag) 
	{
		sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
		depth_to_laser::depth2laser(depth_msg, scan_msg, param);
		ob_pub.publish(scan_msg);
	}
}

// kinect2/hd/image_color_rect  - (1920,1080) - rectified  
// kinect2/qhd/image_color_rect - (960,540)   - rectified
// kinect2/sd/image_color_rect  - (512,424)   - distorted
int main(int argc, char** argv)
{
	//init
    	ros::init(argc, argv, "obkinect");
    	ros::NodeHandle nh;

	//registration
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/kinect2/qhd/image_depth_rect", 1, imageCallback);
	ros::Subscriber caminfo_sub = nh.subscribe<sensor_msgs::CameraInfo>("/kinect2/qhd/camera_info", 1, infoCallback);
    	ob_pub = nh.advertise<sensor_msgs::LaserScan>("/obscan", 1);

	//loop
	ros::spin();
	ros::shutdown();

	return 0;
}











