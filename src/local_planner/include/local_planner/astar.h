#include <cmath>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#define PI 3.14159265359

int distance(int x1, int y1, int x2, int y2)
{
	double dist = sqrt(pow((x1-x2), 2) + pow((y1-y2), 2));
	return dist;
}

double heuristic(double h, double g)
{
	return (h+g);
}

struct Node
{	
	int index;
	int parent;
	double h;
	double g;
	double f = h+g; // h + g
};

int pose_to_index(geometry_msgs::PoseStamped ps, double res, int size_x)
{
	int x_cells = (int)(ps.pose.position.x / res);
	int y_cells = (int)(ps.pose.position.y / res);
	int index = x_cells + size_x*y_cells;
	return index;
}

geometry_msgs::PoseStamped index_to_pose(int i, double res, int size_x)
{	
	geometry_msgs::PoseStamped ps;
	int y_cell = floor(i/size_x);
	int x_cell = i - y_cell*size_x;
	ps.pose.position.x = x_cell*res;
	ps.pose.position.y = y_cell*res;
	return ps;
}

double e_distance(int ind1, int ind2, int width)
{
	int y1 = floor(ind1/width);
	int x1 = ind1 - y1*width;

	int y2 = floor(ind2/width);
	int x2 = ind2 - y2*width;

	int d = distance(x1, y1, x2, y2);
	return d;
}