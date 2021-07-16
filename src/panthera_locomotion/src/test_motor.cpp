#include <iostream>
#include <serial/serial.h>
#include <string>
#include <ros/ros.h>
#include <panthera_locomotion/roboteq.h>

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "roboteq");
	ros::NodeHandle nh;
	RoboteqMotor motor = RoboteqMotor();
	double speed = motor.speed_to_rps(1.0);
	while (true)
	{
		motor.writeSpeed(speed);
		ros::Duration(0.5).sleep();
	}
	ros::spin();
}