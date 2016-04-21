#include <kinect_to_laser_calib.h>

void kinect_to_laser_calibration(sensor_msgs::LaserScan& kinect_scan_msg, const sensor_msgs::LaserScan::ConstPtr& depth_scan_msg, const Params param)
{
	kinect_scan_msg = *depth_scan_msg;

	double ranges_size = depth_scan_msg->ranges.size();
	double kinect_angle_min = depth_scan_msg->angle_min + param.kinect_to_laser_theta_;
	double kinect_angle_max = depth_scan_msg->angle_max + param.kinect_to_laser_theta_;
  	double kinect_angle_increment = (kinect_angle_max - kinect_angle_min) / (ranges_size - 1);

	kinect_scan_msg.angle_min = kinect_angle_min;
	kinect_scan_msg.angle_max = kinect_angle_max;
  	kinect_scan_msg.angle_increment = kinect_angle_increment;
	kinect_scan_msg.ranges.resize(ranges_size);
	kinect_scan_msg.ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());
	kinect_scan_msg.intensities.resize(ranges_size);
	kinect_scan_msg.intensities.assign(ranges_size, 1.0f);

	for (unsigned int i = 0; i < depth_scan_msg->ranges.size(); i++)
	{
		//handle out points out of border 
		double r = depth_scan_msg->ranges[i];
		double theta = depth_scan_msg->angle_min + i * depth_scan_msg->angle_increment;
		double kinect_x = -r * sin(theta + param.kinect_to_laser_theta_) + param.kinect_to_laser_x_ + param.laser_to_robot_x_;
		double kinect_z =  r * cos(theta + param.kinect_to_laser_theta_) + param.kinect_to_laser_z_ + param.laser_to_robot_z_;

		double kinect_th = -atan2(kinect_x, kinect_z);
		if (!std::isfinite(kinect_th) || isnan(kinect_th) || kinect_th < kinect_scan_msg.angle_min || kinect_th > kinect_scan_msg.angle_max) continue;

		double kinect_r = sqrt(pow(kinect_x, 2.0) + pow(kinect_z, 2.0));

		// Determine if this point should be used.
		if (kinect_r > kinect_scan_msg.range_min && kinect_r < kinect_scan_msg.range_max)
		{
			int kinect_index = (kinect_th - kinect_scan_msg.angle_min) / kinect_scan_msg.angle_increment;
			kinect_scan_msg.ranges[kinect_index] = kinect_r;
		}
	}
}

void fusion(sensor_msgs::LaserScan kinect_scan_msg, sensor_msgs::LaserScan laser_scan_msg, sensor_msgs::LaserScan& fusion_scan_msg, const Params param)
{
	fusion_scan_msg = laser_scan_msg;
	if (kinect_scan_msg.range_max > fusion_scan_msg.range_max) 
		ROS_ERROR("RANGE_MAX INVERT!");
	for (unsigned int i = 0; i < kinect_scan_msg.ranges.size(); i++)
	{
		//confidence
		if (i>0 && i<kinect_scan_msg.ranges.size()-1)
		{
			if (abs(kinect_scan_msg.ranges[i+1]-kinect_scan_msg.ranges[i])>param.confidence_T_
				&& abs(kinect_scan_msg.ranges[i-1]-kinect_scan_msg.ranges[i])>param.confidence_T_)
				continue;
		}

		double theta = kinect_scan_msg.angle_min + i * kinect_scan_msg.angle_increment;
		double kinect_r = kinect_scan_msg.ranges[i];
		if (kinect_r > fusion_scan_msg.range_min && kinect_r < fusion_scan_msg.range_max)
		{
			int fusion_index = (theta - fusion_scan_msg.angle_min) / fusion_scan_msg.angle_increment;
			if (kinect_r < fusion_scan_msg.ranges[fusion_index])
				fusion_scan_msg.ranges[fusion_index] = kinect_r;
		}
	}
}

void setParams(Params& param, double kinect_to_laser_x, double kinect_to_laser_y, double kinect_to_laser_z, double kinect_to_laser_theta, 
		double laser_to_robot_x, double laser_to_robot_y, double laser_to_robot_z, double confidence_T)
{
	param.kinect_to_laser_x_ = kinect_to_laser_x;
	param.kinect_to_laser_y_ = kinect_to_laser_y;
	param.kinect_to_laser_z_ = kinect_to_laser_z;
	param.kinect_to_laser_theta_ = kinect_to_laser_theta;

	param.laser_to_robot_x_ = laser_to_robot_x;
	param.laser_to_robot_y_ = laser_to_robot_y;
	param.laser_to_robot_z_ = laser_to_robot_z;

	param.confidence_T_ = confidence_T;
	
}
