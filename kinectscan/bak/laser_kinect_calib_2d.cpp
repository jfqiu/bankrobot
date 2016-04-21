#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

/*
    y                  
    ^   .z           .z
    |  .            .
    | .            .
<----             ----->
x                      x
  (PCL)          (HOKUYO)
*/

//calib
double kinect_to_laser_x_;
double kinect_to_laser_y_;
double kinect_to_laser_z_;
double kinect_to_laser_theta_;
double laser_to_robot_x_;
double laser_to_robot_y_;
double laser_to_robot_z_;

//publisher
ros::Publisher kinect_scan_pub;
ros::Publisher hokuyo_pub;

// show only
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) 
{
	sensor_msgs::LaserScanPtr scan_robot(new sensor_msgs::LaserScan());
	*scan_robot = *scan_msg;
	scan_robot->header.frame_id = "robot";
	scan_robot->intensities.assign(scan_robot->ranges.size(), 0.0);
	hokuyo_pub.publish(scan_robot);
}

// depth image subscribe
void depthCallback(const sensor_msgs::LaserScan::ConstPtr& depth_scan_msg)
{
	//calib laserscan & depthscan
	sensor_msgs::LaserScanPtr kinect_scan_msg(new sensor_msgs::LaserScan());
	*kinect_scan_msg = *depth_scan_msg;

	double ranges_size = depth_scan_msg->ranges.size();
	double kinect_min_x = -kinect_scan_msg->range_max * sin(depth_scan_msg->angle_min) + kinect_to_laser_x_ + laser_to_robot_x_;
	double kinect_min_z =  kinect_scan_msg->range_max * cos(depth_scan_msg->angle_min) + kinect_to_laser_z_ + laser_to_robot_z_;
	double kinect_max_x = -kinect_scan_msg->range_max * sin(depth_scan_msg->angle_max) + kinect_to_laser_x_ + laser_to_robot_x_;
	double kinect_max_z =  kinect_scan_msg->range_max * cos(depth_scan_msg->angle_max) + kinect_to_laser_z_ + laser_to_robot_z_;
	double kinect_angle_min = -atan2(kinect_min_x, kinect_min_z) + kinect_to_laser_theta_;
	double kinect_angle_max = -atan2(kinect_max_x, kinect_max_z) + kinect_to_laser_theta_;
  	double kinect_angle_increment = (kinect_angle_max - kinect_angle_min) / (ranges_size - 1);

	kinect_scan_msg->angle_min = kinect_angle_min;
	kinect_scan_msg->angle_max = kinect_angle_max;
  	kinect_scan_msg->angle_increment = kinect_angle_increment;
	kinect_scan_msg->ranges.resize(ranges_size);
	kinect_scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());
	kinect_scan_msg->intensities.resize(ranges_size);
	kinect_scan_msg->intensities.assign(ranges_size, 1.0f);

	for (unsigned int i = 0; i < depth_scan_msg->ranges.size(); i++)
	{
		//handle out points out of border 
		double r = depth_scan_msg->ranges[i];
		double theta = depth_scan_msg->angle_min + i * depth_scan_msg->angle_increment;
		double kinect_x = -r * sin(theta + kinect_to_laser_theta_) + kinect_to_laser_x_ + laser_to_robot_x_;
		double kinect_z =  r * cos(theta + kinect_to_laser_theta_) + kinect_to_laser_z_ + laser_to_robot_z_;

		double kinect_th = -atan2(kinect_x, kinect_z);
		if (!std::isfinite(kinect_th) || isnan(kinect_th) || kinect_th < kinect_scan_msg->angle_min || kinect_th > kinect_scan_msg->angle_max) continue;

		double kinect_r = sqrt(pow(kinect_x, 2.0) + pow(kinect_z, 2.0));

		// Determine if this point should be used.
		if (kinect_r > kinect_scan_msg->range_min && kinect_r < kinect_scan_msg->range_max)
		{
			int kinect_index = (kinect_th - kinect_scan_msg->angle_min) / kinect_scan_msg->angle_increment;
			kinect_scan_msg->ranges[kinect_index] = kinect_r;
		}
	}
	
	kinect_scan_pub.publish(kinect_scan_msg);
}

int main(int argc, char** argv)
{
	//init
    	ros::init(argc, argv, "laser_kinect_calib_2d");
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
   	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
   	ros::Subscriber depthscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/depthscan", 1, depthCallback);
    	kinect_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/kinectscan", 1);
    	hokuyo_pub = nh.advertise<sensor_msgs::LaserScan>("/hokuyoscan", 1);

	//loop
	ros::spin();
	ros::shutdown();

	return 0;
}











