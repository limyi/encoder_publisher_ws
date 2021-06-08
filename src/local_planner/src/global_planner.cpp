#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <local_planner/astar.h>
#include <vector>
#include <algorithm>

class GlobalPlanner
{
private:
	ros::Subscriber pose_sub, occ_grid, simple_goal;
	ros::Publisher global_path;

	std::vector<Node> open_list, closed_list;

	// goal
	geometry_msgs::PoseStamped curr_pose, goal;
	int goal_index;
	bool goal_reached=true;
	std::vector<geometry_msgs::PoseStamped> plan;

	// start
	int start_index;

	// map info
	double res;
	int width, height;
	std::vector<signed char> data_pts;

	// global path
	std::vector<geometry_msgs::PoseStamped> g_path;
	nav_msgs::Path path;

public:
	GlobalPlanner(ros::NodeHandle *nh)
	{
		pose_sub = nh->subscribe("/initialpose", 100, &GlobalPlanner::RobotPose, this);
		occ_grid = nh->subscribe("/occupancy_wayarea", 100, &GlobalPlanner::OccGrid, this);
		simple_goal = nh->subscribe("/move_base_simple/goal", 100, &GlobalPlanner::SimpleGoal, this);
		global_path = nh->advertise<nav_msgs::Path>("global_path", 100);
	}

	void RobotPose(const geometry_msgs::PoseWithCovarianceStamped& msg)
	{
		curr_pose.pose = msg.pose.pose;
		//std::cout << curr_pose << std::endl;
	}

	void OccGrid(const nav_msgs::OccupancyGrid& msg)
	{
		res = msg.info.resolution;
		width = msg.info.width;
		height = msg.info.height;
		data_pts = msg.data;
	}

	void SimpleGoal(const geometry_msgs::PoseStamped& msg)
	{
		goal = msg;
		goal_index = pose_to_index(msg, res, width);
		init(curr_pose);
		std::cout << goal_index << std::endl;
	}

	void init(geometry_msgs::PoseStamped st)
	{
		Node start{pose_to_index(st, res, width), pose_to_index(st, res, width), 0, 0};
		open_list.push_back(start);
		start_index = pose_to_index(st, res, width);
		std::cout << start_index << std::endl;
		printf("Start initialized!\n");
	}

	std::vector<int> get_neighbours(int cell)
	{
		int x_index = index_to_x(cell, width);
		int y_index = index_to_y(cell, width);

		int neighbour;
		std::vector<int> neighbours;
		for (int i=-1; i<=1; i++)
		{
			for (int j=-1; j<=1; j++)
			{
				if ((x_index+j < width) && (x_index+j >= 0) && (y_index+i < height) && (y_index+i >= 0))
				{
					neighbour = coordinates_to_index(x_index+j, y_index+i, width);
					neighbours.push_back(neighbour);
				}
			}
		}
		return neighbours;
	}

	void makePlan(std::vector<signed char> map)
	{	
		Node current = open_list[0];
		open_list.erase(open_list.begin());
		closed_list.push_back(current);
		printf("starting search!\n");
		if (goal_check(current) == true)
		{
			Node nd = current;
			g_path.insert(g_path.begin(), index_to_pose(nd.index, res, width));
			while (nd.index != start_index)
			{
				nd.index = current.parent;
				geometry_msgs::PoseStamped ind = index_to_pose(nd.index, res, width);
				g_path.insert(g_path.begin(), ind);
			}
			path.poses = g_path;
			global_path.publish(path);
		}

		else
		{
			std::vector<int> neighbours = get_neighbours(current.index);
			for (int i : neighbours)
			{	
				for (auto node : closed_list)
				{
					if (node.index == i || map[i] == 100)
					{	
						printf("Cell occupied!\n");
						std::remove(neighbours.begin(), neighbours.end(), i);
					}
				
					Node x{i, current.index, e_distance(i, current.index, width), map[i], e_distance(i, current.index, width) + map[i]};
					printf("Map index checkpoint!\n");
					if (node.f > x.f)
					{
						node.f = x.f;
						node.parent = current.index;
						printf("Updating Node\n");
					}

					else
					{
						open_list.push_back(x);
						printf("Addded Node to open_list\n");
					}
				}
			}
			std::sort(open_list.begin(), open_list.end(), compareF);
		}

	}

	bool goal_check(Node goal)
	{
		return (goal.index == goal_index);
	}

	static bool compareF(const Node& a, const Node& b)
	{
		return (a.f > b.f);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "global_planner_node");
	ros::NodeHandle nh;
	GlobalPlanner gp = GlobalPlanner(&nh);
	ros::spin();
	return 0;
}