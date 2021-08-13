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
	double width, length;
	float side_spacing, fb_spacing;
	bool combined;

	// lidar map
	//std::vector<signed char> lidar_map;
	std::vector<signed char> ult_map, map_data;
	double res;
	int map_height, map_width;
	nav_msgs::OccupancyGrid mp;

	// sonar readings (in cells)
	long int sonar_readings[10][2];

	// ultrasonic sensor positions
	//int back_left, back_right, left_back, left_mid, left_front, front_right, front_left, right_back, right_mid, right_front;
	//float c[2], bl[2],br[2], lb[2],lm[2],lf[2], fl[2],fr[2], rb[2],rm[2],rf[2];
	double sonar_pos[10][2];
	int init=0;

public:
	SonarCostmap(ros::NodeHandle *nh)
	{
		fused_cmap = nh->advertise<nav_msgs::OccupancyGrid>("/fused_cmap", 1000);
		lidar_costmap = nh->subscribe("/semantics/costmap_generator/occupancy_grid", 100, &SonarCostmap::map_callback, this);
		sonar_sensors = nh->subscribe("/ultrasonic_data", 100, &SonarCostmap::sonar_callback, this);
		robot_width = nh->subscribe("/can_encoder", 100, &SonarCostmap::width_callback, this);

		length = nh->param("/robot_length", 2.2);
		width = nh->param("robot_width", 1.0);
		side_spacing = nh->param("/side_spacing", 0.3);
		fb_spacing = nh->param("/fb_spacing", 0.5);
		combined = nh->param("/combined", false);
	}

	void map_callback(const nav_msgs::OccupancyGrid& msg)
	{
		//lidar_map = msg.data;
		res = msg.info.resolution;
		map_width = msg.info.width;
		map_height = msg.info.height;
		map_data = msg.data;
		mp = msg;

		if (init == 0)
		{
			init_ult_sensors(); // initi ultrasonic sensor positions
			printf("Initialized\n");
			init = 1;
			ult_map.resize(map_width*map_height, 0); // size ultrasonic occ grid map to same as occ grid
		}
	}

	void sonar_callback(const local_planner::Sonar& msg)
	{	
		// distance in cells
		// left ultrasonic sensors
		sonar_readings[0][0] = coordinates_to_index(sonar_pos[0][0], sonar_pos[0][1] + len_to_cell(msg.left_b), map_width); // index of object detected
		sonar_readings[0][1] = len_to_cell(msg.left_b); // distance in cells
		sonar_readings[1][0] = coordinates_to_index(sonar_pos[1][0], sonar_pos[1][1] + len_to_cell(msg.left_m), map_width);
		sonar_readings[1][1] = len_to_cell(msg.left_m);
		sonar_readings[2][0] = coordinates_to_index(sonar_pos[2][0], sonar_pos[2][1] + len_to_cell(msg.left_f), map_width);
		sonar_readings[2][1] = len_to_cell(msg.left_f);

		// right ultrasonic sensors
		sonar_readings[3][0] = coordinates_to_index(sonar_pos[3][0], sonar_pos[3][1] - len_to_cell(msg.right_b), map_width);
		sonar_readings[3][1] = len_to_cell(msg.right_b);
		sonar_readings[4][0] = coordinates_to_index(sonar_pos[4][0], sonar_pos[4][1] - len_to_cell(msg.right_m), map_width);
		sonar_readings[4][1] = len_to_cell(msg.right_m);
		sonar_readings[5][0] = coordinates_to_index(sonar_pos[5][0], sonar_pos[5][1] - len_to_cell(msg.right_f), map_width);
		sonar_readings[5][1] = len_to_cell(msg.right_f);

		// back ultrasonic sensors
		sonar_readings[6][0] = coordinates_to_index(sonar_pos[6][0] - len_to_cell(msg.back_l), sonar_pos[6][1], map_width);
		sonar_readings[6][1] = len_to_cell(msg.back_l);
		sonar_readings[7][0] = coordinates_to_index(sonar_pos[7][0] - len_to_cell(msg.back_r), sonar_pos[7][1], map_width);
		sonar_readings[7][1] = len_to_cell(msg.back_r);

		// front ultrasonic sensors
		sonar_readings[8][0] = coordinates_to_index(sonar_pos[8][0] + len_to_cell(msg.front_l), sonar_pos[8][1], map_width);
		sonar_readings[8][1] = len_to_cell(msg.front_l);
		sonar_readings[9][0] = coordinates_to_index(sonar_pos[9][0] + len_to_cell(msg.front_r), sonar_pos[9][1], map_width);
		sonar_readings[9][1] = len_to_cell(msg.front_r);
		
		for (auto i : sonar_readings)
		{
			if (combined == true)
			{
				check_valid(i[0], &map_data, i[1]); // combine sonar readings with lidar occ grid (not tested)
			}
			else
			{
				check_valid(i[0], &ult_map, i[1]); // make occ grid map from sonar readings
			}
		}

		nav_msgs::OccupancyGrid og;
		og.header.frame_id = "velodyne";
		og.info = mp.info;
		if (combined == true)
		{
			og.data = map_data;
		}
		else
		{
			og.data = max(ult_map, map_data); // merge lidar and sonar occ grid
		}
		fused_cmap.publish(og);
		ult_map.clear();
		ult_map.resize(map_width*map_height, 0);
	}

	void width_callback(const geometry_msgs::Twist& msg)
	{
		width = (msg.angular.y + msg.angular.z)/2 + 0.3; // robot witdth incl +0.3 which is combined distance from wheel to cover on both sides (0.15 on each side)
	}

	int len_to_cell(double length)
	{	
		// convert length in (m) to cells
		return (length/res);
	}

	void check_valid(int index, std::vector<signed char>* cmap, int value)
	{
		if (index >= 0 && index < map_width*map_height) // check if index is within valid range
		{	
			// if object detected
			if (value > 0)
			{
				int occ = 100;
				
				// mark surrounding points as occupied
				for (int i=-1; i<=1; i++)
				{
					for (int j=-1; j<=1; j++)
					{
						cmap->at(index+i*map_width+j) = (signed char)occ;
					}
				}
			}
		}
	}

	void init_ult_sensors()
	{	
		double c[2]; // centre of occupancy grid
		c[0] = map_width/2;
		c[1] = map_height/2;

		// sensor positions: sensor_pos[n][0] -> x coordinate | sensor_pos[n][1] -> y coordinate
		// back sensors
		sonar_pos[6][0] = c[0] - (length/2/res);
		sonar_pos[6][1] = c[1] + (fb_spacing/2/res);
		sonar_pos[7][0] = c[0] - (length/2/res);
		sonar_pos[7][1] = c[1] - (fb_spacing/2/res);

		// left sensors
		sonar_pos[0][0] = c[0] - side_spacing/res;
		sonar_pos[0][1] = c[1] + (width/2/res);
		sonar_pos[1][0] = c[0];
		sonar_pos[1][1] = c[1] + (width/2/res);
		sonar_pos[2][0] = c[0] + side_spacing/res;
		sonar_pos[2][1] = c[1] + (width/2/res);

		// front sensors
		sonar_pos[8][0] = c[0] + (length/2/res);
		sonar_pos[8][1] = c[1] + (fb_spacing/2/res);
		sonar_pos[9][0] = c[0] + (length/2/res);
		sonar_pos[9][1] = c[1] - (fb_spacing/2/res);

		// right sensors
		sonar_pos[3][0] = c[0] - side_spacing/res;
		sonar_pos[3][1] = c[1] - (width/2/res);
		sonar_pos[4][0] = c[0];
		sonar_pos[4][1] = c[1] - (width/2/res);
		sonar_pos[5][0] = c[0] + side_spacing/res;
		sonar_pos[5][1] = c[1] - (width/2/res);

		// print out sensors coordinates
		for (auto f : sonar_pos)
		{
			std::cout << f[0] << "," << f[1] << std::endl;
		}
	}

	long int coordinates_to_index(double x, double y, int width)
	{	
		// convert coordinates to index
		long int index = x + y*width;
		return index;
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sonar_costmap_node");
	ros::NodeHandle nh;
	SonarCostmap sc = SonarCostmap(&nh);
	ros::spin();
}