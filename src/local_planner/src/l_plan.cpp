#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <grid_map_msgs/GridMap.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <local_planner/CmapClear.h>

/** PARAMS:
	- length of robot
	- width of robot
	- safety distance
	- clear tolerance (# of occupied squares in search area)

							f3
	l3	+-------------------+---+ l4/f4
		|   				|	|
	l1	+---o---------------o---+ l2
		|	|				|	|
		|	|		x -->	|	|
		|	|				|	|
	r1	+---o---------------o---+ r2 
		|					|	|
	r3	+-------------------+---+ r4/f2
						   f1   
**/

class Robot
{
	private:
		ros::Subscriber CostMap;
		ros::Subscriber RobotPose;
		ros::Publisher CmdVelPub;
		ros::Publisher cmap_clear;

		// Footprint info
		double length, width;
		std::array<int,2> left_back, left_front, right_back, right_front;

		float safety_dist;
		int buffer; // number of squares safety distance
		int clear_tolerance;

		// search area
		int l1,l2,l3,l4,r1,r2,r3,r4,f1,f2,f3,f4;

		// Map info
		int len_x, len_y;
		double res;
		local_planner::CmapClear bools;

		// run once
		int n = 0;

	public:
		Robot(ros::NodeHandle *nh)
		{	
			CostMap = nh->subscribe("/semantics/costmap_generator/occupancy_grid", 1000, &Robot::mapCallback, this);
			CmdVelPub = nh->advertise<geometry_msgs::Twist>("panthera_cmd",100);
			cmap_clear = nh->advertise<local_planner::CmapClear>("check_cmap",100);

			length = nh->param("/robot_length", 1.0);
			width = nh->param("/robot_width", 0.5);
			safety_dist = nh->param("/safety_dist", 0.5);
			clear_tolerance = nh->param("/clear_tolerance", 1);
		}

		void mapCallback(const nav_msgs::OccupancyGrid& msg)
		{	
			len_x = msg.info.width;
			len_y = msg.info.height;
			res = msg.info.resolution;
			if (n == 0)
			{
				fpCoordinates();
				n += 1;
			}
			std::vector<signed char> data_pts = msg.data;
			checkclear(data_pts);
			

		}

		void searchArea()
		{
			// left area search [l1:l2] and [l3:l4]
			l1 = left_back[0] - buffer + left_back[1] * len_x;
			l2 = left_front[0] + buffer + left_front[1] * len_x;
			l3 = left_back[0] - buffer + (left_back[1] + buffer) * len_x;
			l4 = left_front[0] + buffer + (left_front[1] + buffer) * len_x;

			// right area search [r1:r2] and [r3:r4]
			r1 = right_back[0] - buffer + right_back[1] * len_x;
			r2 = right_front[0] + buffer + right_front[1] * len_x;
			r3 = right_back[0] - buffer + (right_back[1] - buffer) * len_x;
			r4 = right_front[0] + buffer + (right_front[1] - buffer) * len_x;

			// front area search [f1:f3] and [f2:f4]
			f1 = right_front[0] + (right_front[1] - buffer) * len_x;
			f2 = right_front[0] + buffer + (right_front[1] - buffer) * len_x;
			f3 = left_front[0] + (left_front[1] + buffer) * len_x;
			f4 = left_front[0] + buffer + (left_front[1] + buffer) * len_x;
			/**
			std::cout << l1 << ' ' << l2 << ' ' << l3 << ' ' << l4 << std::endl;
			std::cout << r1 << ' ' << r2 << ' ' << r3 << ' ' << r4 << std::endl;
			std::cout << f1 << ' ' << f2 << ' ' << f3 << ' ' << f4 << std::endl;
			**/
		}

		void fpCoordinates()
		{
			float centre[2];
			centre[0] = len_x/2;
			centre[1] = len_y/2;

			float horz_dist = width/2; // x axis robot width
			float vert_dist = length/2; // y axis robot length

			int horz_pix = (int)ceil(horz_dist/res);
			int vert_pix = (int)ceil(vert_dist/res);
			buffer = (int)ceil(safety_dist/res);

			left_back[0] = (int)round(centre[0] - vert_pix);
			left_back[1] = (int)ceil(centre[1] + horz_pix);

			left_front[0] = (int)ceil(centre[0] + vert_pix);
			left_front[1] = (int)ceil(centre[1] + horz_pix);

			right_back[0] = (int)round(centre[0] - vert_pix);
			right_back[1] = (int)round(centre[1] - horz_pix);

			right_front[0] = (int)ceil(centre[0] + vert_pix);
			right_front[1] = (int)ceil(centre[1] - horz_pix);

			searchArea();

			//printf("footprinted\n");
			/**
			std::cout << buffer << std::endl;
			std::cout << horz_pix << " " << vert_pix << std::endl;
			std::cout << left_back[0] << " " << left_back[1] << std::endl;
			std::cout << right_back[0] << " " << right_back[1] << std::endl;
			std::cout << left_front[0] << " " << left_front[1] << std::endl;
			std::cout << right_front[0] << " " << right_front[1] << std::endl;
			**/
		}

		void checkclear(std::vector<signed char> cmap)
		{	

			bool left_clear=true, right_clear=true, up_clear=true;
			// left clear
			int l=0, r=0,u=0;
			for (int i = l1; i <= l3; i+=len_x)
			{
				for (int j = i; j <= i + (l2 - l1); j++)
				{
					if (cmap[j] > 0)
					{
						//left_clear = false;
						l += 1;
						if (l>=clear_tolerance)
						{
							goto endleft;
						}
						//break;
					}
				}
			}
			endleft:

			// right clear
			for (int i = r3; i <= r1; i+=len_x)
			{
				for (int j = i; j <= i + (r2 - r1); j++)
				{
					if (cmap[j] > 0)
					{
						r+=1;
						if (r>=clear_tolerance)
						{
							goto endright;
						}
					}
				}
			}
			endright:

			// up clear
			for (int i = f1; i <= f2; i+=len_x)
			{
				for (int j = i; j <= i + (f2-f1); j++)
				{
					if (cmap[j] > 0)
					{
						u+=1;
						if (u>=clear_tolerance)
						{
							goto endup;
						}
					}
				}
			}
			endup:
			
			//printf("checked\n");
			if (l>=clear_tolerance)
			{
				left_clear=false;
			}
			if (r>=clear_tolerance)
			{
				right_clear=false;
			}
			if (u>=clear_tolerance)
			{
				up_clear=false;
			}
			auto* bl = &bools;
			bl->right = right_clear;
			bl->left = left_clear;
			bl->up = up_clear;
			cmap_clear.publish(*bl);
			std::cout << *bl << std::endl;
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_planner_node");
	ros::NodeHandle nh;

	Robot Panthera = Robot(&nh);
	ros::spin();
	return 0;
}