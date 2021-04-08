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

#define PI 3.14159265359

class StateMachine
{
private:
	ros::Subscriber check;
	ros::Subscriber bot_pose;
	ros::Subscriber goal;
	ros::Subscriber width_sub;
	ros::Publisher twist_pub;
	ros::ServiceClient lb_stat, rb_stat, lf_stat, rf_stat;
	int curr_state=1, prev_state=2, dir=0;

	bool left_clear, right_clear, up_clear, radius_clear;

	float forward_limit, horizontal_limit, step; // how far to move
	double start_x=0, start_y=0, curr_x, curr_y;
	bool finished_step;

	geometry_msgs::Twist twist_msg;

	// speed
	float vx = 0.1;
	float wz = 0.1;

	float width;
	float length;

	int n = 0;
	bool operation = true;

	// goal
	bool reached_goal = true;
	double goal_x, goal_y;
	float goal_stop = 0.5;
	bool goal_sent = false;
	double goal_angle;

	int rotation_not_clear = 0;

public:
	StateMachine(ros::NodeHandle *nh)
	{	
		bot_pose = nh->subscribe("/ndt_pose", 1000, &StateMachine::poseCheck, this);
		check = nh->subscribe("check_cmap", 1000, &StateMachine::cmap_check, this);
		goal = nh->subscribe("/move_base_simple/goal", 1000, &StateMachine::goal_location, this);
		width_sub = nh->subscribe("/can_encoder", 1000, &StateMachine::read_width, this);
		twist_pub = nh->advertise<geometry_msgs::Twist>("panthera_cmd", 100);

		lb_stat = nh->serviceClient<panthera_locomotion::Status>("lb_steer_status");
		rb_stat = nh->serviceClient<panthera_locomotion::Status>("rb_steer_status");
		lf_stat = nh->serviceClient<panthera_locomotion::Status>("lf_steer_status");
		rf_stat = nh->serviceClient<panthera_locomotion::Status>("rf_steer_status");
		/**
		lb_stat.waitForExistence();
		rb_stat.waitForExistence();
		lf_stat.waitForExistence();
		rf_stat.waitForExistence();
		**/
		length = nh->param("/robot_length", 1.5);
		horizontal_limit = nh->param("/horizontal_limit", 3.0);
		forward_limit = nh->param("/forward_limit", 3.0);

	}

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
		tf::Quaternion quat;
		tf::quaternionMsgToTF(goal_wz, quat);
		double roll, pitch, yaw;
    	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    	goal_angle = yaw;
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
			int count = 0;
			while (signal == false || count<2 )
			{
				lb_stat.call(lb_req);
				rb_stat.call(rb_req);
				lf_stat.call(lf_req);
				rf_stat.call(rf_req);
				signal = ((bool)lb_req.response.status && (bool)lf_req.response.status && (bool)rb_req.response.status && (bool)rf_req.response.status);
				std::cout << "Signal: " << signal << std::endl;
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
		else{}

	}
	
	// Robot pose subscriber
	void poseCheck(const geometry_msgs::PoseStamped& msg)
	{
		curr_x = msg.pose.position.x;
		curr_y = msg.pose.position.y;
		geometry_msgs::Quaternion curr_q = msg.pose.orientation;
		tf::Quaternion quat;
		tf::quaternionMsgToTF(curr_q, quat);
		double roll, pitch, yaw;
    	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    	double curr_wz = yaw;
		
		// init start_x and start_y to calculate forward distance travelled
		if (n == 0)
		{
			start_x = curr_x;
			start_y = curr_y;
			step = forward_limit;
			n++;
		}
		if (goal_check(curr_x,curr_y) == false && goal_sent == true)
		{
			double dist = sqrt(pow(curr_x-start_x,2) + pow(curr_y-start_y, 2));
			std::cout << dist << " | " << step << std::endl;

			if (dist < step)
			{
				finished_step = false;
			}
			else
			{
				finished_step = true;
			}
			
			sm(curr_x, curr_y);
			
		}
		else
		{	
			if (goal_orientation(curr_wz) == true)
			{
				stop();
				curr_state = 1;
				prev_state = 2;
				goal_sent = false;
			}
			else
			{	
				stop();
				if (rotation_not_clear <= 10)
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
		}
	}

	// Check costmap if clear to move left/right/up/rotate
	void cmap_check(const local_planner::CmapClear& msg)
	{
		right_clear = msg.right;
		left_clear = msg.left;
		up_clear = msg.up;
		radius_clear = msg.radius;
	}

	/** State machine: 
		- State 1: moving right
		- State 2: moving up
		- State 3: moving left
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
					start_x = x;
					start_y = y;
					step = forward_limit;
					curr_state = 2;
					prev_state = 1;
					up();
					dir = curr_state;
				}
				else
				{
					if (dir!=curr_state)
					{
						right();
						dir = curr_state;
					}
				}
			}
			else if (prev_state == 3)
			{
				if (up_clear == 1 || finished_step == true)
				{
					stop();
					start_x = x;
					start_y = y;
					step = forward_limit;
					curr_state = 2;
					prev_state = 3;
					dir = curr_state;
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
				if (finished_step == true || up_clear == false)
				{
					stop();
					curr_state = 3;
					prev_state = 2;
					start_x = x;
					start_y = y;
					step = horizontal_limit;
				}
				else if (up_clear == true && finished_step == false)
				{
					if (dir!=curr_state)
					{
						up();
						dir = curr_state;
					}
				}
			}
			else if (prev_state == 3)
			{
				if (finished_step == true || up_clear == false)
				{
					stop();
					curr_state = 1;
					prev_state = 2;
					start_x = x;
					start_y = y;
					step = horizontal_limit;
				}
				else if (up_clear == true && finished_step == false)
				{
					if (dir!=curr_state)
					{
						up();
						dir = curr_state;
					}
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
					start_x = x;
					start_y = y;
					step = forward_limit;
					curr_state = 2;
					prev_state = 3;
				}
				else if (left_clear == true && finished_step == false)
				{
					if (dir!=curr_state)
					{
						left();
						dir = curr_state;
					}
				}
			}
			else if (prev_state == 1)
			{
				if (up_clear == 1 || finished_step == true)
				{
					stop();
					start_x = x;
					start_y = y;
					step = forward_limit;
					curr_state = 2;
					prev_state = 1;
				}
				else if (left_clear == 0)
				{
					stop();
				}
				else if (left_clear == true && finished_step == false)
				{
					if (dir!=curr_state)
					{
						left();
						dir = curr_state;
					}
				}
			}
		}
	}
	// Check if robot has reached goal
	bool goal_check(double x, double y)
	{
		double dist = sqrt(pow(goal_x-x,2) + pow(goal_y-y, 2));
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

	///////////////// Orientate robot to desired goal pose ///////////////////////////
	bool goal_orientation(double wz)
	{
		if (std::abs(std::abs(wz) - std::abs(goal_angle)) > (5/180*PI))
		{	
			if (radius_clear == true)
			{
				if (wz >= 0)
				{
					if (goal_angle >= wz-PI && goal_angle <= wz)
					{
						rotate_right();
					}
					else
					{
						rotate_left();
					}
				}
				else
				{
					if (goal_angle <= wz+PI && goal_angle >= wz)
					{
						rotate_left();
					}
					else
					{
						rotate_right();
					}
				}
				return false;
			}
			else
			{
				stop();
				return false;
			}
		}
		else
		{
			stop();
			return true;
		}
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
		twist_pub.publish(*ts);

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
		twist_pub.publish(*ts);

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
		twist_pub.publish(*ts);

		check_steer();

		ts->angular.y = vx;
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

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist_pub_node");
	ros::NodeHandle nh;
	StateMachine panthera_sm = StateMachine(&nh);
	ros::spin();
	return 0;
}