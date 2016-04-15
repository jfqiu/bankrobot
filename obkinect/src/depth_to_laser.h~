#ifndef DEPTH_TO_LASER_H
#define DEPTH_TO_LASER_H

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

struct Params
{	
	//camera info
	double _f;
	double _c_u;
	double _c_v;

	//projection info
	int _scan_height;

	//coordinate info
	double _tx;
	double _tz;

	void setParams(double f, double cu, double cv, int scan_height, double tx, double tz)
	{
		_f = f;
		_c_u = cu;
		_c_v = cv;

		_scan_height = scan_height;

		_tx = tx;
		_tz =  tz;		
	}
};

namespace depth_to_laser 
{
	bool use_point(const float new_value, const float old_value, const float range_min, const float range_max);
	void depth2laser(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::LaserScanPtr scan_msg, const Params param);
}


#endif
