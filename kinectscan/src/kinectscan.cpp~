#include <ros/ros.h>

#include <stdio.h>
#include <iostream>
#include <depth_to_laser.h>
#include <depth_traits.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  

/*
    y                  
    ^   .z           .z
    |  .            .
    | .            .
<----             ----->
x                      x
  (PCL)          (HOKUYO)
*/

//laserscan 
double PI = 3.141592653;
double kinect_to_laser_x_;
double kinect_to_laser_y_;
double kinect_to_laser_z_;
double kinect_to_laser_theta_;
double laser_to_robot_x_;
double laser_to_robot_y_;
double laser_to_robot_z_;

//subscribe 
Params param;
std::vector<geometry_msgs::Pose2D> laserpt;
bool camera_info_ready = false;
bool laser_ready = false;
bool kinect_ready = false;

//publisher
ros::Publisher kinectscan_pub;

// camera info subscribe
void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	if (!camera_info_ready)
	{
		param.setParams(info_msg->K[0], info_msg->K[2], info_msg->K[5], info_msg->height,	
				 kinect_to_laser_x_, kinect_to_laser_y_, kinect_to_laser_z_, kinect_to_laser_theta_,
				 laser_to_robot_x_, laser_to_robot_y_, laser_to_robot_z_);
		camera_info_ready = true;
	}
}

int laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg) 
{
	// laserScan recieve
	for (unsigned int i = 0; i < laserscan_msg->ranges.size(); i++)
	{
		//handle out points out of border 
		double r = laserscan_msg->ranges[i];
		double theta = laserscan_msg->angle_min + i * laserscan_msg->angle_increment;
		if (r > laserscan_msg->range_min && r < laserscan_msg->range_max)
		{
			double x = -r * sin(theta);
			double z =  r * cos(theta);
		}
	}

	return 0;
}

// depth image subscribe
void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
	if (camera_info_ready) 
	{
		// depth_to_laser
		sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
		depth_to_laser::depth2laser(depth_msg, scan_msg, param);
		//if (laser_ready) // default: laserscan faster than depthscan
		{
			depth_to_laser::depth_laser_fusion(scan_msg, laserpt, param);
			kinectscan_pub.publish(scan_msg);
			laser_ready = false;
		}
	}
}

// kinect2/hd/image_color_rect  - (1920,1080) - rectified  
// kinect2/qhd/image_color_rect - (960,540)   - rectified
// kinect2/sd/image_color_rect  - (512,424)   - distorted
int main(int argc, char** argv)
{
	//init
    	ros::init(argc, argv, "kinectscan");
    	ros::NodeHandle nh;

	//params
  	nh.param("kinect_to_laser_x", kinect_to_laser_x_, kinect_to_laser_x_);
  	nh.param("kinect_to_laser_y", kinect_to_laser_y_, kinect_to_laser_y_);
  	nh.param("kinect_to_laser_z", kinect_to_laser_z_, kinect_to_laser_z_);
  	nh.param("kinect_to_laser_theta", kinect_to_laser_theta_, kinect_to_laser_theta_);
  	nh.param("laser_to_robot_x", laser_to_robot_x_, laser_to_robot_x_);
  	nh.param("laser_to_robot_y", laser_to_robot_y_, laser_to_robot_y_);
  	nh.param("laser_to_robot_z", laser_to_robot_z_, laser_to_robot_z_);

	//registration
	image_transport::ImageTransport it_depth(nh);
	image_transport::Subscriber sub_depth = it_depth.subscribe("/kinect2/sd/image_depth_rect", 1, depthCallback);
	ros::Subscriber caminfo_sub = nh.subscribe<sensor_msgs::CameraInfo>("/kinect2/sd/camera_info", 1, infoCallback);
    	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
    	kinectscan_pub = nh.advertise<sensor_msgs::LaserScan>("/kinectscan", 1);

	//loop
	ros::spin();
	ros::shutdown();

	return 0;
}











