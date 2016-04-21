#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define scale 50
#define PI    3.141592653

bool flag = false;
int my_callback(const sensor_msgs::LaserScan::ConstPtr& scan_rec) 
{
	// params show
	if (!flag) 
	{
		printf("frame_id: %s\n", scan_rec->header.frame_id.c_str());
		printf("angle_range: [%f, %f]\n", scan_rec->angle_min, scan_rec->angle_max);
		printf("angle_increment: %f\n", scan_rec->angle_increment);	
		printf("distance_range: [%f, %f]\n", scan_rec->range_min, scan_rec->range_max);
		printf("points size: %ld\n", scan_rec->ranges.size());
		flag = true;
	}

	// laserScan recieve
	int WIDTH = scan_rec->range_max * 2 * scale;
	int HEIGHT = scan_rec->range_max * scale;
	cv::Mat src(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0,0,0));

	double x, z;
	unsigned int n = scan_rec->ranges.size();
	for (unsigned int i = 0; i < n; i++)
	{
		//handle out points out of border 
		double r = scan_rec->ranges[i];
		double theta = scan_rec->angle_min + i * scan_rec->angle_increment;
		if (r > scan_rec->range_min && r < scan_rec->range_max)
		{
			x = -scale * r * sin(theta);
			z =  scale * r * cos(theta);
		}
		cv::Point centerpoint1, centerpoint2;
		centerpoint1.x = x + WIDTH / 2;
		centerpoint1.y = HEIGHT - z;
		centerpoint2.x = WIDTH / 2;
		centerpoint2.y = HEIGHT;
		cv::circle(src, centerpoint1, 1, cv::Scalar(255,255,255));
	}

	// display
	cv::imshow("laserScan", src);
	cv::waitKey(1);
	return 0;
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "display");
    	ros::NodeHandle n;

	cv::namedWindow("laserScan");
	cv::startWindowThread();

    	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/kinectscan", 1, my_callback);

	ros::spin();  
	cv::destroyWindow("laserScan");  
	ros::shutdown();

	return 0;
}
