#ifndef _KINECT_TO_LASER_CALIB_H
#define _KINECT_TO_LASER_CALIB_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

struct Params
{	
	double kinect_to_laser_x_;
	double kinect_to_laser_y_;
	double kinect_to_laser_z_;
	double kinect_to_laser_theta_;

	double laser_to_robot_x_;
	double laser_to_robot_y_;
	double laser_to_robot_z_;

	double confidence_T_;
};

void setParams(Params& param, double kinect_to_laser_x, double kinect_to_laser_y, double kinect_to_laser_z, double kinect_to_laser_theta, 
		double laser_to_robot_x, double laser_to_robot_y, double laser_to_robot_z, double confidence_T);

void kinect_to_laser_calibration(sensor_msgs::LaserScan& kinect_scan_msg, const sensor_msgs::LaserScan::ConstPtr& depth_scan_msg, const Params param);
void fusion(sensor_msgs::LaserScan kinect_scan_msg, sensor_msgs::LaserScan laser_scan_msg, sensor_msgs::LaserScan& fusion_scan_msg, const Params param);

#endif
