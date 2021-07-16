#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <stdio.h>

class RoboteqMotor
{

private :
	int ratio = 1000;
	float wheel_diameter = 0.3;

public :
	RoboteqMotor()
	{	
		serial::Serial ser;
		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		ser.setPort("/dev/ttyACM0");
		ser.setBaudrate(115200);
		ser.setTimeout(timeout);
		ser.open();
	}
	void writeSpeed(double rps)
	{
		double speed = rps * ratio;
		auto spd = "!G " + std::to_string(speed) + "\r";
		//ser.write(spd);
		std::cout << spd << std::endl;
	}

	double speed_to_rps(double speed)
	{
		return speed/wheel_diameter;
	}

};