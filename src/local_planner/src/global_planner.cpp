#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <local_planner/astar.h>
#include <vector>

class GlobalPlanner
{
private:
	ros::Subscriber pose_sub, occ_grid, simple_goal;
	ros::Publisher global_path;

	std::vector<Node> open_list, closed_list;

	geometry_msgs::PoseStamped curr_pose, goal;

	// map info
	double res;
	int width, height;
	std::vector<signed char> data_pts;

public:
	GlobalPlanner(ros::NodeHandle *nh)
	{
		pose_sub = nh->subscribe("/ndt_pose", 100, &GlobalPlanner::RobotPose, this);
		occ_grid = nh->subscribe("/occupancy_grid", 100, &GlobalPlanner::OccGrid, this);
		simple_goal = nh->subscribe("/move_base_simple/goal", 100, &GlobalPlanner::SimpleGoal, this);
		global_path = nh->advertise<nav_msgs::Path>("global_path", 100);
	}

	void RobotPose(const geometry_msgs::PoseStamped& msg)
	{
		curr_pose = msg;
		init(msg);
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
	}

	void init(geometry_msgs::PoseStamped st)
	{
		Node start{pose_to_index(st, res, width), pose_to_index(st, res, width), 0, 0};
		open_list.push_back(start);
	}

	void expand_node(Node n)
	{
		std::vector<int> neighbours{n.index+width-1, n.index+width, n.index+width+1,
									n.index-1,						n.index+1,
									n.index-width-1, n.index-width, n.index-width+1};
		for (int i : neighbours)
		{
			Node x{i, n.index, e_distance(n.index, i, width), data_pts[i]};
		}
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