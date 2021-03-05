#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cmath>

#define PI 3.1415926535897
class TwistFilter
{
	public:
	TwistFilter(ros::NodeHandle *nh)
	{	
		ros::Rate rate(1);
		linear_x = 0;
		angular_z = 0;
		width = 0.7;
		pub = nh->advertise<geometry_msgs::Twist>("/panthera_cmd", 100);
		sub = nh->subscribe("/twist_cmd", 1000, &TwistFilter::read_cmd, this);
		width_sub = nh->subscribe("/can_encoder", 1000, &TwistFilter::read_width, this);
		/**
		while (ros::ok())
		{	
			TwistFilter::run();
			rate.sleep();
		}**/
	}

	double rad_to_deg(double rad)
	{
		double deg = (rad/PI) * 180;

		return deg;
	}

	struct Angles
		{
			double lb, rb, lf, rf;
		};

	auto adjust_wheels(double vx, double wz)
	{	
		width = TwistFilter::get_width;
		Angles ang;
		double radius = 0.0;

		if(wz == 0){
			radius = std::numeric_limits<double>::infinity();
		}else{
			radius = vx/wz;
		}

		double left = radius - width/2;
		double right = radius + width/2;

		ang.lf = TwistFilter::rad_to_deg(atan(length*0.5/left));
		ang.rf = TwistFilter::rad_to_deg(atan(length*0.5/right));
		ang.lb = -ang.lf;
		ang.rb = -ang.rf;

		return ang;
	}
	double get_width()
	{
		std::cout << width << std::endl;
		return width;
	}

	void read_width(const geometry_msgs::Twist& msg)
	{
		width = (msg.angular.y + msg.angular.z)/2;
	}

	void read_cmd(const geometry_msgs::Twist& msg)
	{
		linear_x = msg.linear.x;
		angular_z = msg.angular.z;
		std::cout << linear_x << std::endl;
		//std::cout << width << std::endl;
		Angles a = TwistFilter::adjust_wheels(linear_x, angular_z);
		geometry_msgs::Twist ts;
		ts.linear.x = a.lb;
		ts.linear.y = a.rb;
		ts.linear.z = a.lf;
		ts.angular.x = a.rf;
		ts.angular.y = linear_x;
		ts.angular.z = angular_z;

		pub.publish(ts);
		//TwistFilter::run();
	}

	void run()
	{	
		Angles a = adjust_wheels(linear_x, angular_z);
		geometry_msgs::Twist ts;
		ts.linear.x = a.lb;
		ts.linear.y = a.rb;
		ts.linear.z = a.lf;
		ts.angular.x = a.rf;
		ts.angular.y = linear_x;
		ts.angular.z = angular_z;

		pub.publish(ts);
	}

	protected:
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::Subscriber width_sub;

	double linear_x;
	double angular_z;

	double width;
	const double length = 1.31;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "panthera_twist_node");
	ros::NodeHandle nh;
	TwistFilter panthera = TwistFilter(&nh);

	ros::spin();
	return 0;
}