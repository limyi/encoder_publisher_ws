#include <ros/ros.h>
#include <local_planner/CmapClear.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <cmath>
#include <panthera_locomotion/Status.h>

class StateMachine
{
private:
	ros::Subscriber check;
	ros::Subscriber bot_pose;
	ros::Publisher twist_pub;
	ros::ServiceClient lb_stat, rb_stat, lf_stat, rf_stat;
	int curr_state=1, prev_state=2;

	bool left_clear, right_clear, up_clear;

	float step=0.5; // how far forward to move
	double start_x=0, start_y=0, curr_x, curr_y;
	bool finished_step;

	geometry_msgs::Twist twist_msg;

	// speed
	float vx = 0.1;

	int n = 0;
	bool operation = false;

public:
	StateMachine(ros::NodeHandle *nh)
	{	
		bot_pose = nh->subscribe("/ndt_pose", 1000, &StateMachine::poseCheck, this);
		check = nh->subscribe("check_cmap", 1000, &StateMachine::cmap_check, this);
		twist_pub = nh->advertise<geometry_msgs::Twist>("panthera_cmd", 100);
		lb_stat = nh->serviceClient<panthera_locomotion::Status>("lb_steer_status");
		rb_stat = nh->serviceClient<panthera_locomotion::Status>("rb_steer_status");
		lf_stat = nh->serviceClient<panthera_locomotion::Status>("lf_steer_status");
		rf_stat = nh->serviceClient<panthera_locomotion::Status>("rf_steer_status");
	}

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

	void poseCheck(const geometry_msgs::PoseStamped& msg)
	{
		curr_x = msg.pose.position.x;
		curr_y = msg.pose.position.y;
		if (n == 0)
		{
			start_x = curr_x;
			start_y = curr_y;
			n++;
		}

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

	void stop()
	{
		auto* ts = &twist_msg;
		ts->angular.y = 0;
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