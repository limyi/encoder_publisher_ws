#include <ros/ros.h>
#include <local_planner/CmapClear.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <cmath>

class StateMachine
{
private:
	ros::Subscriber check;
	ros::Subscriber bot_pose;
	ros::Publisher twist_pub;
	int curr_state=1, prev_state=2;

	bool left_clear, right_clear, up_clear;

	float step=0.5; // how far forward to move
	double start_x, start_y, curr_x, curr_y;
	bool finished_step;

	geometry_msgs::Twist twist_msg;

	// speed
	float vx = 0.1;

public:
	StateMachine(ros::NodeHandle *nh)
	{	
		bot_pose = nh->subscribe("/ndt_pose", 1000, &StateMachine::poseCheck, this);
		check = nh->subscribe("check_cmap", 1000, &StateMachine::cmap_check, this);
		twist_pub = nh->advertise<geometry_msgs::Twist>("panthera_cmd", 100);
	}

	void poseCheck(const geometry_msgs::PoseStamped& msg)
	{
		curr_x = msg.pose.position.x;
		curr_y = msg.pose.position.y;

		double dist = sqrt(pow(curr_x-start_x,2) + pow(curr_y-start_y, 2));
		if (dist < step)
		{
			finished_step = false;
		}
		else
		{
			finished_step = true;
		}
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

	void cmap_check(const local_planner::CmapClear& msg)
	{
		right_clear = msg.right;
		left_clear = msg.left;
		up_clear = msg.up;
	}

	void sm(double x, double y)
	{
		switch(curr_state)
		{	
			// moving right
			case 1:
				switch(prev_state)
				{	
					// from moving up
					case 2:
						switch(right_clear)
						{
							case true:
								break;
							case false:
								stop();
								prev_state = curr_state;
								curr_state = 2;
								start_x = x;
								start_y = y;
						}
					// from moving left
					case 3:
						switch(up_clear)
						{
							case true:
								prev_state = curr_state;
								curr_state = 2;
								start_x = x;
								start_y = y;
							case false:
								break;
						}
						
				}
			//moving up
			case 2:
				switch(prev_state)
				{	
					// from moving right
					case 1:
						switch(up_clear)
						{
							case true:
								if (finished_step == true)
								{
									prev_state = curr_state;
									curr_state = 3;
								}
								else
								{
									break;
								}
							case false:
								stop();
								prev_state = curr_state;
								curr_state = 3;
						}
					// from moving left
					case 3:
						switch(up_clear)
						{
							case true:
								break;
							case false:
								stop();
								prev_state = curr_state;
								curr_state = 1;
						}
						
				}
			// moving left
			case 3:
				switch(prev_state)
				{	
					// from moving up
					case 2:
						switch(left_clear)
						{
							case true:
								break;
							case false:
								stop();
								prev_state = curr_state;
								curr_state = 2;
								start_x = x;
								start_y = y;
						}
					// from moving right
					case 1:
						switch(up_clear)
						{
							case true:
								stop();
								prev_state = curr_state;
								curr_state = 2;
								start_x = x;
								start_y = y;
							case false:
								break;
						}
				}
		}
	}



	void right()
	{
		auto* ts = &twist_msg;
		ts->linear.x = -90;
		ts->linear.y = -90;
		ts->linear.z = -90;
		ts->angular.x = -90;
		ts->linear.x = vx;
		twist_pub.publish(*ts);
	}

	void left()
	{
		auto* ts = &twist_msg;
		ts->linear.x = 90;
		ts->linear.y = 90;
		ts->linear.z = 90;
		ts->angular.x = 90;
		ts->linear.x = vx;
		twist_pub.publish(*ts);
	}

	void up()
	{
		auto* ts = &twist_msg;
		ts->linear.x = 0;
		ts->linear.y = 0;
		ts->linear.z = 0;
		ts->angular.x = 0;
		ts->linear.x = vx;
		twist_pub.publish(*ts);
	}

	void stop()
	{
		auto* ts = &twist_msg;
		ts->linear.x = 0;
		twist_pub.publish(*ts);
		ros::Duration(2).sleep();
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist_pub_node");
	ros::NodeHandle nh;
	ros::spin();
	return 0;
}