#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopTurtle   
{
public:
	TeleopTurtle();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
	ros::NodeHandle nh_;

  	int linear_, angular_;
  	int linear_up_, linear_down_, angular_up_, angular_down_;

	float scale_l_, scale_r_;

  	ros::Publisher vel_pub_;
  	ros::Subscriber joy_sub_;
};

TeleopTurtle::TeleopTurtle(): linear_(7), angular_(6), linear_up_(3), linear_down_(0), angular_up_(1), angular_down_(2)
{
  	nh_.param("axis_linear", linear_, linear_);
  	nh_.param("axis_angular", angular_, angular_);
  	nh_.param("up_linear", linear_up_, linear_up_);
  	nh_.param("down_linear", linear_down_, linear_down_);
  	nh_.param("up_angular", angular_up_, angular_up_);
  	nh_.param("down_angular", angular_down_, angular_down_);

	scale_l_ = 0.2f;
	scale_r_ = 0.2f;

  	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 5, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  	geometry_msgs::Twist vel;

	if (joy->buttons[linear_up_]) scale_l_ += 0.2;
	if (joy->buttons[linear_down_] && scale_l_ >= 0.2 * 2) scale_l_ -= 0.2;
	if (joy->buttons[angular_up_]) scale_r_ += 0.2;
	if (joy->buttons[angular_down_] && scale_r_ >= 0.2 * 2) scale_r_ -= 0.2;

 	vel.linear.x = scale_l_ * joy->axes[linear_];
  	vel.angular.z = scale_r_ * joy->axes[angular_];
  	vel_pub_.publish(vel);
	ROS_INFO("JOY -- linear: %f*[%f] angular: %f*[%f]", scale_l_, joy->axes[linear_], scale_r_, joy->axes[angular_]);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "joycontrol");
	TeleopTurtle teleop_turtle;

	ros::spin();
}
