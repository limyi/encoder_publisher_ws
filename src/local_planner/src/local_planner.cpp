#include <ros/ros.h>
#include <local_planner/CmapClear.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <cmath>
#include <panthera_locomotion/Status.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <local_planner/twist_pub_node.h>
#include <autoware_msgs/LaneArray.h>

/**
Things to edit:
1. Extract global path (done)
2. Iterate through global path (done)
3. Adjust robot pose at each step (done)
4. Add limits into state machine 
**/

#define PI 3.14159265359

class LocalPlanner
{
private:
	ros::Subscriber check;
	ros::Subscriber bot_pose;
	ros::Subscriber goal;
	ros::Subscriber width_sub;
	ros::Subscriber global_path_sub;
	ros::Publisher twist_pub;

	ros::ServiceClient lb_stat, rb_stat, lf_stat, rf_stat;

	// state machine
	int curr_state=1, prev_state=2;

	// costmap clearance
	bool left_clear, right_clear, up_clear, radius_clear, back_clear;

	float forward_limit, horizontal_limit, step; // how far forward to move
	double start_x=0, start_y=0, curr_x, curr_y, curr_t;
	bool finished_step;

	geometry_msgs::Twist twist_msg;

	// speed
	float vx = 0.1;
	float wz = 0.5;
	int rotating;

	float width;
	float length;

	int n = 0;
	bool operation = false;

	// goal
	bool reached_goal = true;
	double goal_x, goal_y;
	float goal_stop;
	bool goal_sent = false;
	double pose_tolerance;

	int rotation_not_clear = 0;

	// trying to align
	bool align_attempt = false;

	double delta_theta;

	// Global path
	std::vector<geometry_msgs::PoseStamped> global_path;

public:
	LocalPlanner(ros::NodeHandle *nh)
	{	
		bot_pose = nh->subscribe("/ndt_pose", 1000, &LocalPlanner::poseCheck, this);
		check = nh->subscribe("/check_cmap", 1000, &LocalPlanner::cmap_check, this);
		goal = nh->subscribe("/move_base_simple/goal", 1000, &LocalPlanner::goal_location, this);
		width_sub = nh->subscribe("/can_encoder", 1000, &LocalPlanner::read_width, this);
		global_path_sub = nh->subscribe("/lane_waypoints_array", 1000, &LocalPlanner::get_path, this);
		twist_pub = nh->advertise<geometry_msgs::Twist>("panthera_cmd", 100);

		lb_stat = nh->serviceClient<panthera_locomotion::Status>("lb_steer_status");
		rb_stat = nh->serviceClient<panthera_locomotion::Status>("rb_steer_status");
		lf_stat = nh->serviceClient<panthera_locomotion::Status>("lf_steer_status");
		rf_stat = nh->serviceClient<panthera_locomotion::Status>("rf_steer_status");

		length = nh->param("/robot_length", 1.5);
		goal_stop = nh->param("/goal_stop", 0.5);
		forward_limit = nh->param("/forward_limit", 0.5);
		horizontal_limit = nh->param("/horizontal_limit", 6.0);
		delta_theta = nh->param("/delta_theta", 10);
		pose_tolerance = nh->param("/pose_tolerance", 5);
	}

	// get global path (edit to add points if dtheta is more than certain angle)
	void get_path(const autoware_msgs::LaneArray& msg)
	{
		geometry_msgs::PoseStamped pt = msg.lanes[0].waypoints[0].pose;
		global_path.push_back(pt);
		int i = 1;
		while (i<sizeof(msg.lanes[0].waypoints)/sizeof(msg.lanes[0].waypoints[0]))
		{
			if (std::abs(angle_diff(quat_to_rad(pt,"rad"), quat_to_rad(msg.lanes[0].waypoints[i].pose, "rad"), "deg")) >= delta_theta)
			{
				global_path.push_back(msg.lanes[0].waypoints[i].pose);
				pt = msg.lanes[0].waypoints[i].pose;
			}
			i++;
		}
		if(std::find(global_path.begin(), global_path.end(), global_path[global_path.size()-1]) != global_path.end())
		{
		   global_path.push_back(global_path[global_path.size()-1]);
		} 
	}

	// move to first wp


	// Wheel separation of robot
	void read_width(const geometry_msgs::Twist& msg)
	{
		width = (msg.angular.y + msg.angular.z)/2;
	}

	// Subscribe goal location
	void goal_location(const geometry_msgs::PoseStamped& msg)
	{	
		goal_x = msg.pose.position.x;
		goal_y = msg.pose.position.y;
		geometry_msgs::Quaternion goal_wz = msg.pose.orientation;
		goal_sent = true;
	}

	// Check if wheels have adjusted to correct angle
	void check_steer()
	{
		if (operation == true)
		{
			panthera_locomotion::Status lb_req,rb_req,lf_req,rf_req;
			lb_req.request.reconfig = true;
			rb_req.request.reconfig = true;
			lf_req.request.reconfig = true;
			rf_req.request.reconfig = true;
			bool signal = false;
			ros::Rate rate(2);
			while (signal == false)
			{
				lb_stat.call(lb_req);
				rb_stat.call(rb_req);
				lf_stat.call(lf_req);
				rf_stat.call(rf_req);
				signal = (lb_req.response.status && lf_req.response.status && rb_req.response.status && rf_req.response.status);
				std::cout << "Signal: " << signal << std::endl;
				rate.sleep();
			}
		}
		else{}

	}
	
	// Robot pose subscriber
	void poseCheck(const geometry_msgs::PoseStamped& msg)
	{
		curr_x = msg.pose.position.x;
		curr_y = msg.pose.position.y;
		curr_t = quat_to_rad(msg);
		
		// init start_x and start_y
		if (n == 0)
		{
			start_x = curr_x;
			start_y = curr_y;
			n++;
		}

		int aligned = align_pose(curr_t, global_path[0]);

		if (goal_check(curr_x, curr_y, global_path[1]) == false && goal_sent == true)
		{	
			double dist = sqrt(pow(curr_x-start_x,2) + pow(curr_y-start_y, 2));
			std::cout << dist << " " << curr_state << std::endl;
			if (dist < step)
			{
				finished_step = false;
			}
			else
			{
				finished_step = true;
			}
			
			if (aligned == 1)
			{
				if (rotating != aligned)
				{
					rotate_left();
					rotating = aligned;
				}
			}
			else if (aligned == -1)
			{
				if (rotating != aligned)
				{
					rotate_right();
					rotating = aligned;
				}
			}
			else if (aligned == 0)
			{
				if (rotating != aligned)
				{
					stop();
					rotating = aligned;
				}
				else
				{
					sm(curr_x, curr_y);

					switch(curr_state)
					{
						case 1:
							right();
						case 2:
							up();
						case 3:
							left();
					}
				}
			}
			else if (align_attempt == true && radius_clear == true)
			{
				if (aligned == 1)
				{
					rotate_left();
					rotating = aligned;
				}
				else if (aligned == -1)
				{
					rotate_right();
					rotating = aligned;
				}
				else
				{
					stop();
					global_path.erase(global_path.begin());
					align_attempt = false;
				}
			}
		}
		else
		{	
			if (aligned == 0)
			{
				stop();
				curr_state = 1;
				prev_state = 2;
				global_path.erase(global_path.begin());
			}
			else if (aligned == 1)
			{
				if (rotating != aligned)
				{	
					// rotate
					if (radius_clear == 0)
					{
						rotate_left();
						rotating = aligned;
					}
					else
					{
						if (curr_state == 1 && left_clear == true)
						{
							left();
							align_attempt = true;
						}
						else if (curr_state == 3 && right_clear == true)
						{
							right();
							align_attempt = true;
						}
						else if (curr_state == 2 && back_clear == true)
						{
							reverse();
							align_attempt = true;
						}
						else
						{
							stop();
							printf("Help I'm stuck\n");
						}
					}
				}
			}
			else if (aligned == -1)
			{
				if (rotating != aligned)
				{
					rotate_right();
					rotating = aligned;
				}
			}
			else if (aligned == 0)
			{
				stop();
				align_attempt = false;
				global_path.erase(global_path.begin());
			}

			if (global_path.size() <= 1)
			{
				goal_sent = false;
			}
			/**
			else
			{	
				stop();
				if (rotation_not_clear <= 100)
				{
					std::cout << "rotation not clear" << std::endl;
					rotation_not_clear++;
					ros::Rate rate(1);
					rate.sleep();
				}
				else
				{
					std::cout << "Unable to rotate error" << std::endl;
					goal_sent = false;
				}
			}
			**/
		}
	}

	// Check costmap if clear to move left/right/up/rotate
	void cmap_check(const local_planner::CmapClear& msg)
	{
		right_clear = msg.right;
		left_clear = msg.left;
		up_clear = msg.up;
		back_clear = msg.back;
		radius_clear = msg.radius;
	}

	// align robot when at start of goal and at end of each horizontal movement
	int align_pose(double current, geometry_msgs::PoseStamped goal)
	{	
		double diff = angle_diff(current, quat_to_rad(goal, "rad"), "deg");
		if (diff >= pose_tolerance)
		{
			return -1;
		}
		else if (diff <= -pose_tolerance)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}

	/** State machine: 
		- State 1: moving right
		- State 2: moving up
		- State 3: moving left
	
	void sm(double x, double y)
	{	
		// moving right
		if (curr_state == 1)
		{
			if (prev_state == 2)
			{
				if (right_clear == 0)
				{
					stop();
					start_x = x;
					start_y = y;
					curr_state = 2;
					prev_state = 1;
				}
			}
			else if (prev_state == 3)
			{
				if (up_clear == 1)
				{
					stop();
					start_x = x;
					start_y = y;
					curr_state = 2;
					prev_state = 3;
				}
				else if (right_clear == 0)
				{
					stop();
				}
			}
		}
		// moving up
		else if (curr_state == 2)
		{
			if (prev_state == 1)
			{
				if (finished_step == 1)
				{
					stop();
					curr_state = 3;
					prev_state = 2;
				}
				else if (up_clear == false)
				{
					stop();
					curr_state = 3;
					prev_state = 2;
				}
			}
			else if (prev_state == 3)
			{
				if (finished_step == 1)
				{
					stop();
					curr_state = 1;
					prev_state = 2;
				}
				else if (up_clear == false)
				{
					stop();
					curr_state = 1;
					prev_state = 2;
				}
			}
		}
		// moving left
		else if (curr_state == 3)
		{
			if (prev_state == 2)
			{
				if (left_clear == 0)
				{
					stop();
					start_x = x;
					start_y = y;
					curr_state = 2;
					prev_state = 3;
				}
			}
			else if (prev_state == 1)
			{
				if (up_clear == 1)
				{
					stop();
					start_x = x;
					start_y = y;
					curr_state = 2;
					prev_state = 1;
				}
				else if (left_clear == 0)
				{
					stop();
				}
			}
		}
	}
	**/
	void sm(double x, double y)
	{	
		// moving right
		if (curr_state == 1)
		{
			if (prev_state == 2)
			{
				if (right_clear == 0 || finished_step == true)
				{
					stop();
					curr_state = 2;
					prev_state = 1;
					start_x = x;
					start_y = y;
					step = horizontal_limit;
				}
			}
			else if (prev_state == 3)
			{
				if (up_clear == 1 || finished_step == true)
				{
					stop();
					curr_state = 2;
					prev_state = 3;
					start_x = x;
					start_y = y;
					step = forward_limit;
				}
				else if (right_clear == 0)
				{
					stop(); // stuck
				}
			}
		}
		// moving up
		else if (curr_state == 2)
		{
			if (prev_state == 1)
			{
				if (finished_step == 1 || up_clear == false)
				{
					stop();
					curr_state = 3;
					prev_state = 2;
					start_x = x;
					start_y = y;
					step = horizontal_limit;
				}
			}
			else if (prev_state == 3)
			{
				if (finished_step == 1 || up_clear == false)
				{
					stop();
					curr_state = 1;
					prev_state = 2;
					start_x = x;
					start_y = y;
					step = horizontal_limit;
				}
			}

		}
		// moving left
		else if (curr_state == 3)
		{
			if (prev_state == 2)
			{
				if (left_clear == 0 || finished_step == true)
				{
					stop();
					curr_state = 2;
					prev_state = 3;
					start_x = x;
					start_y = y;
					step = forward_limit;
				}
			}
			else if (prev_state == 1)
			{
				if (up_clear == 1)
				{
					stop();
					curr_state = 2;
					prev_state = 1;
					start_x = x;
					start_y = y;
					step = forward_limit;
				}
				else if (left_clear == 0)
				{
					stop(); // stuck
				}
			}
		}
	}

	// Check if robot has reached goal
	bool goal_check(double x, double y, geometry_msgs::PoseStamped goal)
	{
		double dist = sqrt(pow(goal.pose.position.x - x, 2) + pow(goal.pose.position.y - y, 2));
		if (dist < goal_stop)
		{
			reached_goal = true;
		}
		else
		{
			reached_goal = false;
		}
		//std::cout << "reached goal: " << reached_goal << std::endl;
		return reached_goal;
	}

	//////////////// Velocity Commands ///////////////////////////

	void right()
	{
		auto* ts = &twist_msg;
		ts->linear.x = -90;
		ts->linear.y = -90;
		ts->linear.z = -90;
		ts->angular.x = -90;
		ts->angular.y = 0;

		check_steer();

		ts->angular.y = vx;
		ts->angular.z = 0;
		twist_pub.publish(*ts);
	}

	void left()
	{
		auto* ts = &twist_msg;
		ts->linear.x = 90;
		ts->linear.y = 90;
		ts->linear.z = 90;
		ts->angular.x = 90;
		ts->angular.y = 0;

		check_steer();

		ts->angular.y = vx;
		twist_pub.publish(*ts);
	}

	void up()
	{
		auto* ts = &twist_msg;
		ts->linear.x = 0;
		ts->linear.y = 0;
		ts->linear.z = 0;
		ts->angular.x = 0;
		ts->angular.y = 0;

		check_steer();

		ts->angular.y = vx;
		twist_pub.publish(*ts);
	}

	void reverse()
	{
		auto* ts = &twist_msg;
		ts->linear.x = 0;
		ts->linear.y = 0;
		ts->linear.z = 0;
		ts->angular.x = 0;
		ts->angular.y = 0;

		check_steer();

		ts->angular.y = -vx;
		twist_pub.publish(*ts);
	}

	void stop()
	{
		auto* ts = &twist_msg;
		ts->angular.y = 0;
		twist_pub.publish(*ts);
	}

	void rotate_right()
	{
		Angles a = adjust_wheels(0, -wz, width, length);
		auto* ts = &twist_msg;
		ts->linear.x = a.lb;
		ts->linear.y = a.rb;
		ts->linear.z = a.lf;
		ts->angular.x = a.rf;
		ts->angular.y = 0;
		ts->angular.z = 0;

		check_steer();

		ts->angular.z = -wz;
		twist_pub.publish(*ts);
	}

	void rotate_left()
	{
		Angles a = adjust_wheels(0, wz, width, length);
		auto* ts = &twist_msg;
		ts->linear.x = a.lb;
		ts->linear.y = a.rb;
		ts->linear.z = a.lf;
		ts->angular.x = a.rf;
		ts->angular.y = 0;
		ts->angular.z = 0;

		check_steer();

		ts->angular.z = wz;
		twist_pub.publish(*ts);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "local_planner_node");
	ros::NodeHandle nh;
	LocalPlanner panthera_sm = LocalPlanner(&nh);
	ros::spin();
	return 0;
}