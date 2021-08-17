#include <ros/ros.h>
#include <local_planner/CmapClear.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <cmath>
#include <panthera_locomotion/Status.h>
#include <panthera_locomotion/ICRsearch.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <local_planner/twist_pub_node.h>
#include <autoware_msgs/LaneArray.h>
#include <nav_msgs/Path.h>

#define PI 3.14159265359

class LocalPlanner
{
private:
	ros::Subscriber check;
	ros::Subscriber bot_pose;
	ros::Subscriber goal;
	ros::Subscriber width_sub;
	ros::Subscriber global_path_sub;
	ros::Publisher twist_pub, path_pub, vel_pub;

	ros::ServiceClient lb_stat, rb_stat, lf_stat, rf_stat, rot_angle;

	// state machine
	int curr_state=1, prev_state=2, dir=0;

	// costmap clearance
	bool left_clear, right_clear, up_clear, radius_clear, back_clear;

	float forward_limit, horizontal_limit, step; // how far forward to move
	double start_x=0, start_y=0, curr_x, curr_y, curr_t;
	bool finished_step;

	geometry_msgs::Twist twist_msg;

	// speed
	float vx = 0.085;
	float wz = 0.05;
	float factor;
	int rotating; // check if robot is already rotating
	bool static_turn = false; // check if static steer is allowed

	float width;
	float length;

	int n = 0;
	bool operation = true;

	// goal
	bool reached_goal = true;
	geometry_msgs::PoseStamped final_goal;
	float goal_stop;
	double pose_tolerance;
	int  wp_interval;

	// trying to align
	bool align_attempt = false;
	bool icr = false;

	double delta_theta;

	// Global path
	nav_msgs::Path path;
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
		path_pub = nh->advertise<nav_msgs::Path>("new_global_path", 100);
		vel_pub = nh->advertise<geometry_msgs::Twist>("/reconfig", 100);
		
		lb_stat = nh->serviceClient<panthera_locomotion::Status>("lb_steer_status");
		rb_stat = nh->serviceClient<panthera_locomotion::Status>("rb_steer_status");
		lf_stat = nh->serviceClient<panthera_locomotion::Status>("lf_steer_status");
		rf_stat = nh->serviceClient<panthera_locomotion::Status>("rf_steer_status");
		rot_angle = nh->serviceClient<panthera_locomotion::Status>("rotation_angle"); // angle from icr search
		
		//lb_stat.waitForExistence();
		//rb_stat.waitForExistence();
		//lf_stat.waitForExistence();
		//rf_stat.waitForExistence();
		
		length = nh->param("/robot_length", 1.5); // length of robot
		goal_stop = nh->param("/goal_stop", 0.5); // goal tolerance
		forward_limit = nh->param("/forward_limit", 0.5); // distance robot moves forward
		horizontal_limit = nh->param("/horizontal_limit", 10.0); // distance robot moves horizontally
		delta_theta = nh->param("/delta_theta", 10); // angle tolerance
		pose_tolerance = nh->param("/pose_tolerance", 5); // robot pose tolerance
		wp_interval = nh->param("/wp_interval", 10); // take wp every _ number of waypoints
		operation = nh->param("/operation", true); // true means running with robot
		vx = nh->param("/vx", 0.085);
		wz = nh->param("/wz", 0.05);
		factor = nh->param("/factor", 1.5); // lateral acceleration
		icr = nh->param("/icr_search", false); // false: does not search for optimal icr
	}

	// get global path (edit to add points if dtheta is more than certain angle)
	void get_path(const autoware_msgs::LaneArray& msg)
	{	
		global_path.clear();
		geometry_msgs::PoseStamped pt = msg.lanes[0].waypoints[0].pose;
		global_path.push_back(pt);
		int i = 1, last_pt = 1;

		// get waypoints if they differ by delta_theta deg or every wp_interval
		while (i<msg.lanes[0].waypoints.size())
		{
			if (std::abs(angle_diff(quat_to_rad(pt,"rad"), quat_to_rad(msg.lanes[0].waypoints[i].pose, "rad"), "deg")) >= delta_theta || i == msg.lanes[0].waypoints.size()-1 || i-last_pt >=wp_interval)
			{	
				global_path.push_back(msg.lanes[0].waypoints[i].pose);
				pt = msg.lanes[0].waypoints[i].pose;
				last_pt = i;
			}
			i++;
		}
		path.poses = global_path;
		path.header.frame_id = "/map";
		std::cout << "Path of size " << global_path.size() << " received."<< std::endl;
		path_pub.publish(path);
	}

	// Wheel separation of robot
	void read_width(const geometry_msgs::Twist& msg)
	{	
		if (operation == true) // check if running with physical robot
		{
			width = (msg.angular.y + msg.angular.z)/2;
		}
		else
		{
			width = 0.7;
		}
	}

	// Subscribe goal location from rviz
	void goal_location(const geometry_msgs::PoseStamped& msg)
	{	
		final_goal = msg;
	}

	// Check if wheels have adjusted to correct angle
	void check_steer()
	{
		if (operation == true) // run if operating with robot
		{
			panthera_locomotion::Status lb_req,rb_req,lf_req,rf_req;
			lb_req.request.reconfig = true;
			rb_req.request.reconfig = true;
			lf_req.request.reconfig = true;
			rf_req.request.reconfig = true;
			bool signal = false;
			ros::Rate rate(1);
			int count = 0;
			while (signal == false || count<2 ) // count makes sure the wheel has stopped at the correct angle and just moving past the correct angle
			{
				lb_stat.call(lb_req);
				rb_stat.call(rb_req);
				lf_stat.call(lf_req);
				rf_stat.call(rf_req);
				signal = ((bool)lb_req.response.status && (bool)lf_req.response.status && (bool)rb_req.response.status && (bool)rf_req.response.status); // signal is if all the wheels are algined correctly
				//std::cout << "Signal: " << signal << std::endl;
				rate.sleep();
				if (signal==true)
				{
					count++;
				}
				else
				{
					count = 0;
				}
			}
			printf("Clear!\n");
		}
	}
	
	// Robot pose subscriber
	void poseCheck(const geometry_msgs::PoseStamped& msg)
	{	
		// get current pose
		curr_x = msg.pose.position.x;
		curr_y = msg.pose.position.y;
		curr_t = quat_to_rad(msg);

		panthera_locomotion::ICRsearch angle_req; // service to get robot locomotion cmds

		// init start_x and start_y
		if (n == 0)
		{
			start_x = curr_x;
			start_y = curr_y;
			step=forward_limit;
			n++;
		}
		int aligned;
		if (global_path.size() != 0)
		{	
			int rotate_dir = align_pose(curr_t, global_path[0]); // get rotation direction to align to next point

			if (goal_check(curr_x, curr_y, global_path[1]) == true) // if robot is within goal tolerance to next point in path
			{
				global_path.erase(global_path.begin());
				path.poses = global_path;
				path_pub.publish(path);

				// clean left if path turns right & clean right if path turns left
				// 1: path goes right
				// -1: path goes left
				if (align_pose(curr_t, global_path[0]) == 1)
				{
					curr_state = 3; // move left
					prev_state = 2; // previously moving forward
					dir = 0; // reset state = 0
				}
				else if (align_pose(curr_t, global_path[0]) == -1)
				{
					curr_state = 1; // move right
					prev_state = 2; // previously moving forward
					dir = 0; // reset state = 0
				}
			}

			// if havent reached next point in path
			else
			{
				if (rotate_dir != 0) // if robot needs to rotate
				{	
					if (radius_clear == true) // if robot is able to rotate
					{
						if (rotate_dir == 1) // robot needs to rotate right
						{
							if (rotate_dir != rotating) // if robot is not already rotating
							{	
								stop();
								rotate_right();
								rotating = rotate_dir; // set robot to be already rotating
							}
						}
						else if (rotate_dir == -1) // robot needs to rotate left
						{
							if (rotate_dir != rotating) // if robot is not already rotating
							{	
								stop();
								rotate_left();
								rotating = rotate_dir; // set robot to be already rotating
							}
						}
					}

					// if robot cannot rotate (Work in progress)
					else if (radius_clear == false && back_clear == true)
					{
						if (rotate_dir != rotating)
						{	
							if (icr == true)
							{
								angle_req.request.received_angle = true;
								angle_req.request.turn_angle = angle_diff(curr_t, quat_to_rad(global_path[0], "rad"), "rad"); // calculate angle to rotate
								rot_angle.call(angle_req); // send request
								geometry_msgs::Twist wa = angle_req.response.wheel_angles; // wheel angles to rotate
								geometry_msgs::Twist ws = angle_req.response.wheel_speeds; // wheel speeds to rotate
								if(angle_req.response.feasibility == true) // if possible to rotate
								{
									custom_rotate(wa, ws); // rotate around icr
									rotating = rotate_dir;
								}
								else
								{
									// if cannot rotate try to reverse
									reverse();
									rotating = 4;
								}
							}
							else
							{
								reverse();
								rotating = 4;
							}
						}

					}
				}
				else
				{	
					// prevent robot from oscillating near tolerance
					if (rotating != 0)
					{	
						ros::Duration(1).sleep();
						stop();
						rotating = 0;
						dir = 0;
					}

					// continue normal state machine
					else
					{
						sm(curr_x, curr_y);
					}
				}
			}
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
		//double diff = (current - quat_to_rad(goal, "rad"))/PI*180;
		double diff = angle_diff(current, quat_to_rad(goal, "rad"), "deg");
		std::cout << "Angle diff: " << diff << std::endl;
		if (std::round(diff) >= pose_tolerance)
		{
			return 1; // goal is to the right of current orientation
		}
		else if (std::round(diff) <= -pose_tolerance)
		{
			return -1; // goal is to the left of current orientation
		}
		else
		{
			return 0; // no need to rotate
		}
	}

	void sm(double x, double y)
	{	
		double dist = sqrt(pow(x-start_x,2) + pow(y-start_y, 2));
		std::cout << dist << " | " << step << std::endl;
		// check if robot has reached forward/horizontal limit
		if (dist <= step)
		{
			finished_step = false;
		}
		else
		{
			finished_step = true;
		}
		std::cout << (finished_step == true) << std::endl;
		std::cout << curr_state << " " << prev_state << std::endl;

		// lateral acceleration
		double ratio = 1;//factor - (std::abs(dist - step/2)/(step/2));

		//////////////////////////// MOVING RIGHT ////////////////////////////////////
		if (curr_state == 1)
		{	
			if (prev_state != 3) // if robot was not moving left previously
			{
				if (right_clear == 0 || finished_step == true) // if robot has reached horizontal limit or right not clear
				{	
					stop();
					start_x = x;
					start_y = y;
					if (up_clear == false) // if front not clear, move left
					{
						curr_state = 3;
						prev_state = 1;
						step = horizontal_limit;
					}
					else
					{	
						// if front clear, move forward
						step = forward_limit;
						curr_state = 2;
						prev_state = 1;
					}
				}
				else if (right_clear == true && finished_step == false) // if still possible to move right
				{	
					if (dir!=curr_state && static_turn==true) // wheels steer before moving
					{	
						right(1);
						dir = curr_state;
					}
					else if (static_turn==false) // wheels steer while moving instead of wheels steering then moving
					{
						right(ratio);
					}
				}
			}
			else
			{
				if (up_clear == true || right_clear == false || finished_step == true) // if previously moving left & currently moving right and maxed movement to the right
				{	
					// move forward
					stop();
					start_x = x;
					start_y = y;
					step = forward_limit;
					curr_state = 2;
					prev_state = 3;
				}
				else if (right_clear == true && finished_step == false)
				{	
					if (dir!=curr_state && static_turn==true) // static_turn is steering the wheels if the turn while robot remains stationary
					{	
						right(1);
						dir = curr_state;
					}
					else if (static_turn==false) // robot moves while wheel steer
					{
						right(ratio);
					}
				}
			}
		}
		//////////////////////////// MOVING UP ////////////////////////////////////
		else if (curr_state == 2)
		{	
			if (prev_state == 1) // if robot moving right previously
			{
				if (finished_step == true || up_clear == false) // move left if reached horizontal limit  or obstacle
				{
					stop();
					curr_state = 3;
					prev_state = 2;
					start_x = x;
					start_y = y;
					step = horizontal_limit;
				}
				else if (up_clear == true  && finished_step == false) // continue moving up
				{
					if (dir!=curr_state)
					{
						up();
						dir = curr_state;
					}
				}
			}
			else if (prev_state == 3) // if moving left previously
			{
				if (finished_step == true || up_clear == false) // move right if reached horizontal limit or obstacle
				{
					stop();
					curr_state = 1;
					prev_state = 2;
					start_x = x;
					start_y = y;
					step = horizontal_limit;
				}
				else if (up_clear == true && finished_step == false) // continue moving up
				{
					if (dir!=curr_state)
					{
						up();
						dir = curr_state;
					}
				}
			}
			else
			{
				stop();
			}
		}
		//////////////////////////// MOVING LEFT ////////////////////////////////////
		else if (curr_state == 3)
		{
			if (prev_state != 1) // was not moving right previously
			{
				if (left_clear == false || finished_step == true) // if reached limit or obstacle
				{
					stop();
					start_x = x;
					start_y = y;
					if (up_clear == false) // obstacle in front
					{	
						// move right
						curr_state = 1;
						prev_state = 3;
						step = horizontal_limit;
					}
					else
					{	
						// move forward
						step = forward_limit;
						curr_state = 2;
						prev_state = 3;
					}
				}
				else if (left_clear == true && finished_step == false) // continue moving left
				{
					if (dir!=curr_state && static_turn==true) // publish cmd to move left once & wheels steer before moving off
					{
						left(1);
						dir = curr_state;
					}
					else if (static_turn==false) // wheels steer while moving off
					{
						left(ratio);
					}
				}
			}
			else
			{

				if (up_clear == true || left_clear == false || finished_step == true) // move forward if was moving right previously
				{	
					stop();
					start_x = x;
					start_y = y;
					step = forward_limit;
					curr_state = 2;
					prev_state = 1;
				}
				else if (left_clear == true && finished_step == false) // continue moving left
				{	
					if (dir!=curr_state && static_turn==true) // wheels steer before moving off
					{	
						left(1);
						dir = curr_state;
					}
					else if (static_turn==false) // wheels steer while moving off
					{
						left(ratio);
					}
				}
			}

		}
	}

	// Check if robot has reached goal
	bool goal_check(double x, double y, geometry_msgs::PoseStamped goal)
	{
		double dist = sqrt(pow(goal.pose.position.x - x, 2) + pow(goal.pose.position.y - y, 2));
		if (dist < goal_stop) // if robot is within tolerance distance of goal
		{
			reached_goal = true;
		}
		else
		{
			reached_goal = false;
		}
		return reached_goal;
	}

	//////////////// Velocity Commands ///////////////////////////
	
	void right(double ratio)
	{	
		auto* ts = &twist_msg;
		ts->linear.x = -90;
		ts->linear.y = -90;
		ts->linear.z = -90;
		ts->angular.x = -90;
		//ts->angular.y = 0;
		//twist_pub.publish(*ts);

		//check_steer();

		ts->angular.y = vx * ratio;
		ts->angular.z = 0;
		twist_pub.publish(*ts);
	}

	void left(double ratio)
	{
		auto* ts = &twist_msg;
		ts->linear.x = 90;
		ts->linear.y = 90;
		ts->linear.z = 90;
		ts->angular.x = 90;
		//ts->angular.y = 0;
		//twist_pub.publish(*ts);

		//check_steer();

		ts->angular.y = vx * ratio;
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
		twist_pub.publish(*ts);

		//check_steer();

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
		twist_pub.publish(*ts);

		check_steer();

		ts->angular.y = -vx;
		twist_pub.publish(*ts);
	}

	void stop()
	{	
		printf("stopping\n");
		auto* ts = &twist_msg;
		ts->linear.x = 0;
		ts->linear.y = 0;
		ts->linear.z = 0;
		ts->angular.x = 0;
		ts->angular.y = 0;
		ts->angular.z = 0;
		twist_pub.publish(*ts);

		check_steer();
		ts->angular.y = 0;
		ts->angular.z = 0;
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
		twist_pub.publish(*ts);

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
		twist_pub.publish(*ts);

		check_steer();

		ts->angular.z = wz;
		twist_pub.publish(*ts);
	}

	void custom_rotate(geometry_msgs::Twist wheel_angles, geometry_msgs::Twist wheel_speeds)
	{
		twist_pub.publish(wheel_angles);

		check_steer();

		vel_pub.publish(wheel_speeds);
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