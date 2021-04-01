#include <iostream>

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
