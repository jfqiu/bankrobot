#include <depth_to_laser.h>
#include <depth_traits.h>

bool depth_to_laser::use_point(const float new_value, const float old_value, const float range_min, const float range_max)
{  
	// Check for NaNs and Infs, a real number within our limits is more desirable than these.
	bool new_finite = std::isfinite(new_value);
	bool old_finite = std::isfinite(old_value);

	// Infs are preferable over NaNs (more information)
	if (!new_finite && !old_finite) // Either is NaN or Inf.
	{ 
		if (!isnan(new_value)) // new is +-Inf value.
		{ 
			return true;
		}
		return false; // new is NaN.
	}

	bool range_check = range_min <= new_value && new_value <= range_max;

	// If new is not in range, don't bother
	if (!range_check)
	{
		return false;
	}

	if (!old_finite) // New value is in range and finite, use it.
	{ 
		return true;
	}

	// Finally, if they are both numerical and new_value is closer than old_value, use new_value.
	bool shorter_check = new_value < old_value;
	return shorter_check;
}

void depth_to_laser::depth2laser(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::LaserScanPtr scan_msg, const Params param)
{
	// Calculate vars
	double unit_scaling = depthimage_to_laserscan::DepthTraits<uint16_t>::toMeters(uint16_t(1));
	float constant_x = unit_scaling / param._f;
	uint32_t ranges_size = depth_msg->width;

	double kinect_angle_max = -atan2((double)(0 - param._c_u) * constant_x, unit_scaling);
	double kinect_angle_min = -atan2((double)(depth_msg->width-1 - param._c_u) * constant_x, unit_scaling);
	double kinect_angle_increment = (kinect_angle_max - kinect_angle_min) / (depth_msg->width - 1);

	// Fill the kinect fixed message
	sensor_msgs::LaserScanPtr kinect_scan_msg(new sensor_msgs::LaserScan());
	kinect_scan_msg->header = depth_msg->header;
	kinect_scan_msg->header.frame_id = "kinect2";
	kinect_scan_msg->time_increment = 0.0;
	kinect_scan_msg->scan_time = 0.033;
	kinect_scan_msg->range_min = 0.45;    
	kinect_scan_msg->range_max = 5.0;
	kinect_scan_msg->angle_min = kinect_angle_min;
	kinect_scan_msg->angle_max = kinect_angle_max;
  	kinect_scan_msg->angle_increment = kinect_angle_increment;

	kinect_scan_msg->ranges.resize(ranges_size);
	kinect_scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());
	kinect_scan_msg->intensities.assign(ranges_size, 0.5);

	const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
	int row_step = depth_msg->step / sizeof(uint16_t);

	int offset = (int)(param._c_v-param._scan_height/2);
	depth_row += offset*row_step; 

	for(int v = offset; v < offset+param._scan_height; v++, depth_row += row_step)
	{
		for (int u = 0; u < (int)depth_msg->width; u++) // Loop in row
		{	
			uint16_t depth = depth_row[u];

			double kinect_r = depth; // Assign to pass through NaNs and Infs
			double kinect_th = -atan2((double)(u - param._c_u) * constant_x, unit_scaling); // -atan2(x, z)
			int kinect_index = (kinect_th - kinect_scan_msg->angle_min) / kinect_scan_msg->angle_increment;

			if (depthimage_to_laserscan::DepthTraits<uint16_t>::valid(depth)) // Not NaN or Inf
			{
				// Calculate in XYZ
				double kinect_x = (u - param._c_u) * depth * constant_x;
				double kinect_z = depthimage_to_laserscan::DepthTraits<uint16_t>::toMeters(depth);

				// Calculate actual distance
				kinect_r = sqrt(pow(kinect_x, 2.0) + pow(kinect_z, 2.0));
			}

			// Determine if this point should be used.
			if (use_point(kinect_r, kinect_scan_msg->ranges[kinect_index], kinect_scan_msg->range_min, kinect_scan_msg->range_max))
			{
				kinect_scan_msg->ranges[kinect_index] = kinect_r;
				kinect_scan_msg->intensities[kinect_index] = 0.5;
			}
		}
	}			
	
	// Fill the robot fixed message
	sensor_msgs::LaserScanPtr robot_scan_msg(new sensor_msgs::LaserScan());
	robot_scan_msg->header = depth_msg->header;
	robot_scan_msg->header.frame_id = "bankrobot";
	robot_scan_msg->time_increment = 0.0;
	robot_scan_msg->scan_time = 0.033;
	robot_scan_msg->range_min = 0.45;    
	robot_scan_msg->range_max = 5.0;

	double robot_min_x = -robot_scan_msg->range_max * sin(kinect_scan_msg->angle_min) - param._tx;
	double robot_min_z =  robot_scan_msg->range_max * cos(kinect_scan_msg->angle_min) - param._tz;
	double robot_angle_min = -atan2(robot_min_x, robot_min_z);
	double robot_max_x = -robot_scan_msg->range_max * sin(kinect_scan_msg->angle_max) - param._tx;
	double robot_max_z =  robot_scan_msg->range_max * cos(kinect_scan_msg->angle_max) - param._tz;
	double robot_angle_max = -atan2(robot_max_x, robot_max_z);
  	double robot_angle_increment = (robot_angle_max - robot_angle_min) / (depth_msg->width - 1);

	robot_scan_msg->angle_min = robot_angle_min;
	robot_scan_msg->angle_max = robot_angle_max;
  	robot_scan_msg->angle_increment = robot_angle_increment;
	robot_scan_msg->ranges.resize(ranges_size);
	robot_scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());
	robot_scan_msg->intensities.assign(ranges_size, 0.5);

	for (int i = 0; i < ranges_size; i++)
	{
		double kinect_r = kinect_scan_msg->ranges[i];
		double kinect_th = kinect_angle_min + i * kinect_angle_increment;

		// New point (In the robot coordinate)
		double robot_x = -kinect_r * sin(kinect_th) - param._tx;
		double robot_z =  kinect_r * cos(kinect_th) - param._tz;

		// New angle (In the robot coordinate)
		double robot_th = -atan2(robot_x, robot_z);
		if (!std::isfinite(robot_th) || isnan(robot_th) || robot_th < robot_scan_msg->angle_min || robot_th > robot_scan_msg->angle_max) continue;

		int robot_index = (robot_th - robot_scan_msg->angle_min) / robot_scan_msg->angle_increment;
		double robot_r = sqrt(pow(robot_x, 2.0) + pow(robot_z, 2.0));

		// Determine if this point should be used.
		if (use_point(robot_r, robot_scan_msg->ranges[robot_index], robot_scan_msg->range_min, robot_scan_msg->range_max))
		{
			robot_scan_msg->ranges[robot_index] = robot_r;
			robot_scan_msg->intensities[robot_index] = 0.5;
		}
	}

	/******** kinect_scan_msg[frame_id] = "kinect2" | robot_scan_msg[frame_id] = "bankrobot" ********/
	//*scan_msg = *kinect_scan_msg;
	*scan_msg = *robot_scan_msg;
}


/* 
std_msgs/Header header
float32 angle_min           
float32 angle_max            
float32 angle_increment      
float32 time_increment       
float32 scan_time            
float32 range_min            
float32 range_max            
float32[] ranges             
float32[] intensities
*/

