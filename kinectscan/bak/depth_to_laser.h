#ifndef DEPTH_TO_LASER_H
#define DEPTH_TO_LASER_H

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

struct Params
{	
	//camera info
	double _f;
	double _c_u;
	double _c_v;

	//projection info
	int _scan_height;

	//coordinate info
	double _kinect_to_laser_x;
	double _kinect_to_laser_y;
	double _kinect_to_laser_z;
	double _kinect_to_laser_theta;

	double _laser_to_robot_x;
	double _laser_to_robot_y;
	double _laser_to_robot_z;

	void setParams(double f, double cu, double cv, int scan_height, 
			double kinect_to_laser_x, double kinect_to_laser_y, double kinect_to_laser_z, double kinect_to_laser_theta,
			double laser_to_robot_x, double laser_to_robot_y, double laser_to_robot_z)
	{
		_f = f;
		_c_u = cu;
		_c_v = cv;

		_scan_height = scan_height;

		_kinect_to_laser_x = kinect_to_laser_x;
		_kinect_to_laser_y = kinect_to_laser_y;	
		_kinect_to_laser_z = kinect_to_laser_z;
		_kinect_to_laser_theta = kinect_to_laser_theta;

		_laser_to_robot_x = laser_to_robot_x;	
		_laser_to_robot_y = laser_to_robot_y;
		_laser_to_robot_z = laser_to_robot_z;		
	}
};

namespace depth_to_laser 
{
	bool use_point(const float new_value, const float old_value, const float range_min, const float range_max);
	void depth2laser(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::LaserScanPtr scan_msg, const Params param);
	void depth_laser_fusion(sensor_msgs::LaserScanPtr scan_msg, std::vector<geometry_msgs::Pose2D> laserscan_points, const Params param);
}


#endif
