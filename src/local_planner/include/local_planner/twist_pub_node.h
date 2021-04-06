#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265359

double rad_to_deg(double rad)
{
	double deg = (rad/PI) * 180;

	return deg;
}

struct Angles
{
	double lb, rb, lf, rf;
};

auto adjust_wheels(double vx, double wz, double width, double length)
{	
	Angles ang;
	double radius = 0.0;

	if(wz == 0){
		radius = std::numeric_limits<double>::infinity();
	}else{
		radius = vx/wz;
	}

	double left = radius - width/2;
	double right = radius + width/2;

	ang.lf = rad_to_deg(atan(length*0.5/left));
	ang.rf = rad_to_deg(atan(length*0.5/right));
	ang.lb = -ang.lf;
	ang.rb = -ang.rf;

	return ang;
}

double quat_to_rad(geometry_msgs::PoseStamped ps, std::string unit="rad")
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(ps.pose.orientation, quat);
	double roll, pitch, yaw, goal_angle;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	goal_angle = yaw;
	if (unit == "rad")
	{
		return goal_angle;
	}
	else
	{
		return (goal_angle/PI*180);
	}
}

double angle_diff(double x, double y, std::string unit="rad")
{
	if (x < 0)
	{
		x = 2*PI + x;
	}
	if (y <  0)
	{
		y = 2*PI + y;
	}
	double diff = x-y;
	if (unit=="rad")
	{
		return diff;
	}
	else
	{
		return (diff/PI*180);
	}
}