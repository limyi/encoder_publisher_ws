#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <local_planner/twist_pub_node.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <visualizaion_msgs/Marker.h>

class AreaCovered
{
private :
	ros::Subscriber pose;
	ros::Subscriber world_to_map;
	ros::Publisher occ_grid, marker_pub;

	// world to map
	double map_x, map_y, res;
	int map_width, map_height;

	// new occupancy grid
	nav_msgs::OccupancyGrid map;

	// footprint radius [m]
	float robot_radius = 1.5;

	// counter
	int n=0;

public :
	AreaCovered(ros::NodeHandle *nh)
	{
		pose = nh->subscribe("/initialpose", 1000, &AreaCovered::robot_pose, this);
		world_to_map = nh->subscribe("/occupancy_wayarea", 1000, &AreaCovered::map_origin, this);
		occ_grid = nh->advertise<nav_msgs::OccupancyGrid>("/area_covered", 100);
		marker_pub = nh->advertise<visualizaion_msgs::Marker>("/marker_topic", 100);
	}

	void map_origin(const nav_msgs::OccupancyGrid& msg)
	{	
		// map x-y coordinates wrt world frame
		res = msg.info.resolution;
		map_width = msg.info.width;
		map_height = msg.info.height;
		map_x = msg.info.origin.position.x - map_width/2;
		map_y = msg.info.origin.position.y; - map_height/2;
		if (n == 0)
		{
			map.data = msg.data;
			map.info = msg.info;
			map.header.frame_id = "area_covered_map";
			n++;
		}
	}

	void robot_pose(const geometry_msgs::PoseWithCovarianceStamped& msg)
	{	
		// robot x-y coordinates wrt world frame
		double robot_x = msg.pose.pose.position.x;
		double robot_y = msg.pose.pose.position.y;

		// robot to map x-y coordinates
		double x = robot_x - map_x;
		double y = robot_y - map_y;
		std::vector<int> robot_2_map{(int)x, (int)y};

		// number of cells for the length of the radius
		int num_cells_rad = robot_radius/res;

		auto cell_bound = SearchArea(x, y);

		for (int i = cell_bound[0]; i <= cell_bound[1]; i+=(2*num_cells_rad))
		{
			auto coors = index_to_coor(map.data[i]);
			if (eucl_dist(coors, robot_2_map) < robot_radius)
			{
				if (map.data[i] != 100)
				{
					map.data[i] = 100;
				}
				std::cout << map.data[i] << std::endl;
			}
		}
		occ_grid.publish(map);

	}

	void add_marker(geometry_msgs::PoseWithCovarianceStamped& msg)
	{
		
	}

	std::vector<int> SearchArea(double x, double y)
	{
		// index of bottom left corner
		double bottom_x, bottom_y, top_x, top_y;
		int bottom, top;

		bottom_x = x - robot_radius;
		bottom_y = y - robot_radius;
		top_x = x - robot_radius;
		top_y = y - robot_radius;

		bottom = std::round(bottom_y * map_width + bottom_x);
		top = std::round(top_y * map_width + top_x);

		std::vector<int> corners{bottom, top};

		return corners;

	}

	std::vector<int> index_to_coor(int ind)
	{
		

		std::vector<int> vec{std::floor(ind/map_width), (ind % map_width)};

		return vec;
	}

	double eucl_dist(std::vector<int> a, std::vector<int> b)
	{
		double dist = sqrt(pow((a[0] - b[0]), 2) + pow((a[1] - b[1]), 2));
		return dist;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "area_covered_node");
	ros::NodeHandle nh;
	AreaCovered footprint = AreaCovered(&nh);
	ros::spin();
	return 0;
}