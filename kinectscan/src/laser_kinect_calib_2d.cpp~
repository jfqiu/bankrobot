#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <kinect_to_laser_calib.h>

/*
    y                  
    ^   .z           .z
    |  .            .
    | .            .
<----             ----->
x                      x
  (PCL)          (HOKUYO)
*/

// calib params
Params param;
double kinect_to_laser_x_;
double kinect_to_laser_y_;
double kinect_to_laser_z_;
double kinect_to_laser_theta_;
double laser_to_robot_x_;
double laser_to_robot_y_;
double laser_to_robot_z_;
double confidence_T_;

// publisher
sensor_msgs::LaserScan laser_scan_msg;
sensor_msgs::LaserScan kinect_scan_msg;
sensor_msgs::LaserScan fusion_scan_msg;
ros::Publisher kinect_scan_pub;
ros::Publisher hokuyo_scan_pub;
ros::Publisher fusion_scan_pub;

bool laser_scan_ready = false;
bool kinect_scan_ready = false;

// laser_scan
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) 
{
	laser_scan_msg = *scan_msg;
	laser_scan_msg.header.frame_id = "robot";
	laser_scan_msg.intensities.assign(laser_scan_msg.ranges.size(), 0.0);
	hokuyo_scan_pub.publish(laser_scan_msg);

	// fusionscan
	if (kinect_scan_ready)
	{
		fusion(kinect_scan_msg, laser_scan_msg, fusion_scan_msg, param);
		fusion_scan_pub.publish(fusion_scan_msg);
		kinect_scan_ready = false;
	}	

	//laser_scan_ready = true;
}

// kinect_scan
void depthCallback(const sensor_msgs::LaserScan::ConstPtr& depth_scan_msg)
{
	// calib depth_to_laser
	kinect_to_laser_calibration(kinect_scan_msg, depth_scan_msg, param);
	kinect_scan_pub.publish(kinect_scan_msg);

	kinect_scan_ready = true;
}

int main(int argc, char** argv)
{
	// init
    	ros::init(argc, argv, "laser_kinect_calib_2d");
    	ros::NodeHandle nh;

	// params
  	nh.param("kinect_to_laser_x", kinect_to_laser_x_, kinect_to_laser_x_);
  	nh.param("kinect_to_laser_y", kinect_to_laser_y_, kinect_to_laser_y_);
  	nh.param("kinect_to_laser_z", kinect_to_laser_z_, kinect_to_laser_z_);
  	nh.param("kinect_to_laser_theta", kinect_to_laser_theta_, kinect_to_laser_theta_);
  	nh.param("laser_to_robot_x", laser_to_robot_x_, laser_to_robot_x_);
  	nh.param("laser_to_robot_y", laser_to_robot_y_, laser_to_robot_y_);
  	nh.param("laser_to_robot_z", laser_to_robot_z_, laser_to_robot_z_);
	nh.param("confidence_T", confidence_T_, confidence_T_);
	setParams(param, kinect_to_laser_x_, kinect_to_laser_y_, kinect_to_laser_z_, kinect_to_laser_theta_, 
			laser_to_robot_x_, laser_to_robot_y_, laser_to_robot_z_, confidence_T_);

	// subscriber
   	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
   	ros::Subscriber depthscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/depthscan", 1, depthCallback);

	// publisher
    	kinect_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/kinectscan", 1);
    	hokuyo_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/hokuyoscan", 1);
    	fusion_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/fusionscan", 1);

	// loop
	ros::spin();
	ros::shutdown();

	return 0;
}











