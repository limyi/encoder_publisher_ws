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

	// goal
	geometry_msgs::PoseStamped curr_pose, goal;
	int goal_index;
	bool goal_reached=true;
	std::vector<geometry_msgs::PoseStamped> plan;

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
		search(msg);

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
		goal_index = pose_to_index(msg);
	}

	void init(geometry_msgs::PoseStamped st)
	{
		Node start{pose_to_index(st, res, width), pose_to_index(st, res, width), 0, 0};
		open_list.push_back(start);
	}

	void expand_node(Node n)
	{	
		open_list.erase(open_list.begin());
		std::vector<int> neighbours{n.index+width-1, n.index+width, n.index+width+1,
									n.index-1,						n.index+1,
									n.index-width-1, n.index-width, n.index-width+1};

		for (int i : neighbours)
		{	
			// remove parent node from neighbours to be expanded
			if (i != n.parent)
			{
				// skip squares which are occupied
				if (data_pts[i] < 100)
				{
					Node x{i, n.index, e_distance(n.index, i, width), data_pts[i]};

					// goal check
					if (goal_check(i) == true)
					{
						closed_list.push_back(x);
						goal_reached = true;
					}
					else
					{
						// check for duplicate node with different heuristic
						for (Node nd : closed_list)
						{
							if (x.index == nd.index && x.f < nd.f)
							{	
								open_list.push_back(x);
								open_list.remove(open_list.begin(), open_list.end(), nd);
							}
						}
					}
				}
			}
		}
		std::sort(open_list.begin(), open_list.end(), compareHeuristic);
		closed_list.push_back(n);
	}

	bool compareHeuristic(const Node& a, const Node& b)
	{
		return a.f > b.f;
	}

	bool goal_check(int index)
	{
		return index == goal_index;
	}

	void search(geometry_msgs::PoseStamped& ps)
	{
		open_list.clear();
		closed_list.clear();
		plan.clear();
		init(ps);
		goal_reached = false;
		while (open_list.size() > 0 || goal_reached == false)
		{
			expand_node(open_list[0]);
		}
		makePlan(closed_list[closed_list.size()-1]);
	}

	void makePlan(Node goal)
	{
		int start = pose_to_index(curr_pose);
		int par;
		std::vector<int> path;
		path.push_back(goal.index);
		while (par != start)
		{
			par = goal.parent;
			path.push_front(par);
		}

		for (auto n : path)
		{
			plan.push_back(index_to_pose(n, res, width));
		}
		goal_path.publish(plan);
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