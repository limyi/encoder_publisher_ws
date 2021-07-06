#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <local_planner/Sonar.h>
#include <geometry_msgs/Twist.h>
#include <vector>

class SonarCostmap
{

private:
	ros::Publisher fused_cmap;
	ros::Subscriber sonar_sensors, lidar_costmap, robot_width;

	// robot params
	double width, length, offset_x;
	float side_spacing;

	// lidar map
	std::vector<signed char> lidar_map;
	std::vector<signed char> ult_map;
	double res;
	int height, width;

	// sonar readings (cells)
	int left_b, left_m, left_f, front_l, front_r, right_b, right_m, right_f, back_r, back_l;

public:
	SonarCostmap(ros::NodeHandle *nh)
	{
		fused_cmap = nh->advertise<nav_msgs::OccupancyGrid>("/fused_cmap", 1000);
		lidar_costmap = nh->subscribe("/semantics/costmap_generator/occupancy_grid", 100, &SonarCostmap::map_callback, this);
		sonar_sensors = nh->subscribe("/sonar_sensors", 100, &SonarCostmap::sonar_callback, this);
		robot_width = nh->subscribe("/can_encoder", 100, &SonarCostmap::width_callback, this);

		length = nh->param("/robot_length", 2.2);
		offset_x = nh->param("/offset_x", 0);
		side_spacing = nh->param("/side_spacing", 0.3);
	}

	void map_callback(const nav_msgs::OccupancyGrid& msg)
	{
		lidar_map = msg.data;
	}

	void sonar_callback(const local_planner::Sonar& msg)
	{
		left_b = len_to_cell(msg.left_b);
		left_m = len_to_cell(msg.left_m);
		left_f = len_to_cell(msg.left_f);

		right_b = len_to_cell(msg.right_b);
		right_m = len_to_cell(msg.right_m);
		right_f = len_to_cell(msg.right_f);

		back_l = len_to_cell(msg.back_l);
		back_r = len_to_cell(msg.back_r);

		front_l = len_to_cell(msg.front_l);
		front_r = len_to_cell(msg.front_r);
	}

	void width_callback(const geometry_msgs::Twist& msg)
	{
		width = (msg.angular.y + msg.angular.z)/2;
	}

	int len_to_cell(double length)
	{
		return ((int)std::round(length/lidar_map.info.resolution));
	}

	int detected(int dist)
	{
		if (dist == 0)
		{
			return 0;
		}
		return 1;
	}

	void check_valid(int index, int dist)
	{
		if (index >= 0 && index < lidar_map.info.width*lidar_map.info.height)
		{
			lidar_map.data[index] = detected(dist) * 100;
		}
	}

	void insert_cells()
	{	
		/**
		int centre = std::round(lidar_map.info.height * lidar_map.info.width / 2);

		int lb = centre - std::round(length/2/lidar_map.info.resolution) + (std::round(width/2/lidar_map.info.resolution) + left_b)*lidar_map.info.width;
		int lm = centre + (std::round(width/2/lidar_map.info.resolution) + left_m)*lidar_map.info.width;
		int lf = centre + std::round(length/2/lidar_map.info.resolution) + (std::round(width/2/lidar_map.info.resolution) + left_f)*lidar_map.info.width;

		int rb = centre - std::round(length/2/lidar_map.info.resolution) - (std::round(width/2/lidar_map.info.resolution) + right_b)*lidar_map.info.width;
		int rm = centre + (std::round(width/2/lidar_map.info.resolution) + right_m)*lidar_map.info.width;
		int rf = centre + std::round(length/2/lidar_map.info.resolution) - (std::round(width/2/lidar_map.info.resolution) + right_f)*lidar_map.info.width;

		int fr = centre + std::round(length/2/lidar_map.info.resolution + front_r) - std::round(width/2/lidar_map.info.resolution)*lidar_map.info.width;
		int fl = centre + std::round(length/2/lidar_map.info.resolution + front_l) + std::round(width/2/lidar_map.info.resolution)*lidar_map.info.width;

		int br = centre - std::round(length/2/lidar_map.info.resolution + back_r) - std::round(width/2/lidar_map.info.resolution)*lidar_map.info.width;
		int bl = centre - std::round(length/2/lidar_map.info.resolution + back_l) + std::round(width/2/lidar_map.info.resolution)*lidar_map.info.width;

		check_valid(lb, left_b);
		check_valid(lm, left_m);
		check_valid(lf, left_f);

		check_valid(rb, right_b);
		check_valid(rm, right_m);
		check_valid(rf, right_f);

		check_valid(fr, front_r);
		check_valid(fl, front_l);

		check_valid(bl, back_l);
		check_valid(br, back_r);
		**/
		
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sonar_costmap");
	ros::NodeHandle nh;
	SonarCostmap sc = SonarCostmap(&nh);
	ros::spin();
}