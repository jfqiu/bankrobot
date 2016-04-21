#include <ros/ros.h>

#include <stdio.h>
#include <iostream>
#include <depth_to_laser.h>
#include <depth_traits.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

//subscribe 
Params param;
bool camera_info_ready = false;
bool laser_ready = false;
bool kinect_ready = false;

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

int laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_rec) 
{
	// laserScan recieve
	double x, z;
	for (unsigned int i = 0; i < scan_rec->ranges.size(); i++)
	{
		//handle out points out of border 
		double r = scan_rec->ranges[i];
		double theta = scan_rec->angle_min + i * scan_rec->angle_increment;
		if (r > scan_rec->range_min && r < scan_rec->range_max)
		{
			x = -r * sin(theta);
			z =  r * cos(theta);
		}

		pcl::PointXYZRGB point;
		point.x = -x;
		point.y = 0;
		point.z = z;
		uchar RGB_r = 255, RGB_g = 0, RGB_b = 0;
		uint32_t rgb = (static_cast<uint32_t>(RGB_r) << 16 | static_cast<uint32_t>(RGB_g) << 8 | static_cast<uint32_t>(RGB_b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud->points.push_back (point);
	}
	laser_ready = true;

	return 0;
}

// depth image subscribe
void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
	if (camera_info_ready)
	{
		//3d reconstruction
		double unit_scaling = depthimage_to_laserscan::DepthTraits<uint16_t>::toMeters(uint16_t(1));
		float constant_x = unit_scaling / param._f;
		const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
		int row_step = depth_msg->step / sizeof(uint16_t);	
		for(int v = 0; v < (int)depth_msg->height; v++, depth_row += row_step)
		{
			for (int u = 0; u < (int)depth_msg->width; u++) // Loop in row
			{	
				uint16_t depth = depth_row[u];
				if (depthimage_to_laserscan::DepthTraits<uint16_t>::valid(depth)) // Not NaN or Inf
				{
					double x = (u - param._c_u) * depth * constant_x + kinect_to_laser_x_;
					double y = (v - param._c_v) * depth * constant_x + kinect_to_laser_y_;
					double z = depthimage_to_laserscan::DepthTraits<uint16_t>::toMeters(depth) + kinect_to_laser_z_;  

					pcl::PointXYZRGB point;
					point.x = -x;
					point.y = -y;
					point.z = z;
					uchar RGB_r = 0, RGB_g = 155, RGB_b = 0;
					uint32_t rgb = (static_cast<uint32_t>(RGB_r) << 16 | static_cast<uint32_t>(RGB_g) << 8 | static_cast<uint32_t>(RGB_b));
					point.rgb = *reinterpret_cast<float*>(&rgb);
					cloud->points.push_back (point);
				}
			}
		}	
		kinect_ready = true;
	}
}

// kinect2/hd/image_color_rect  - (1920,1080) - rectified  
// kinect2/qhd/image_color_rect - (960,540)   - rectified
// kinect2/sd/image_color_rect  - (512,424)   - distorted
int main(int argc, char** argv)
{
	//init
    	ros::init(argc, argv, "laser_kinect_calib_3d");
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
	image_transport::Subscriber sub_depth = it_depth.subscribe("/kinect2/qhd/image_depth_rect", 1, depthCallback);
	ros::Subscriber caminfo_sub = nh.subscribe<sensor_msgs::CameraInfo>("/kinect2/qhd/camera_info", 1, infoCallback);
    	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);

	//loop
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	ros::Rate r(10);
	while (ros::ok())
	{
		ros::spinOnce();
		if (laser_ready && kinect_ready)
		{
			viewer.showCloud(cloud);
			cloud->clear();
			laser_ready = false;
			kinect_ready = false;
		}
		r.sleep();
	}
	ros::shutdown();

	return 0;
}











