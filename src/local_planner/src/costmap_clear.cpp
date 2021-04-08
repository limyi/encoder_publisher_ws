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

			b4				f4
	l3	+---+---------------+---+ l4
		|   b3				f3	|
	l1	+---+---------------+---+ l2
		|	|				|	|
		|	|		x -->	|	|
		|	|				|	|
	r1	+---+---------------+---+ r2 
		|	b2				f2	|
	r3	+---+---------------+---+ r4
			b1			   f1   
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
		float offset_x=-0.27, offset_y=0;

		float safety_dist;
		int buffer; // number of squares safety distance
		int clear_tolerance;
		float clear_radius;

		// search area
		int l1,l2,l3,l4,r1,r2,r3,r4,f1,f2,f3,f4,b1,b2,b3,b4;
		std::vector<int> radial_area;

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

			length = nh->param("/robot_length", 2.2);
			width = nh->param("/robot_width", 1.0);
			safety_dist = nh->param("/safety_dist", 0.5);
			clear_tolerance = nh->param("/clear_tolerance", 1);
			clear_radius = nh->param("/clear_radius", 1.0);
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
			f1 = r4 - buffer;
			f2 = r2 - buffer;
			f3 = l2 - buffer;
			f4 = l4 - buffer;

			b1 = r3 + buffer;
			b2 = r1 + buffer;
			b3 = l1 + buffer;
			b4 = l3 + buffer;

			
			std::cout << l3 << ' ' << b4 << ' ' << f4 << ' ' << l4 << std::endl;
			std::cout << l1 << ' ' << b3 << ' ' << f3 << ' ' << l2 << std::endl;
			std::cout << r1 << ' ' << b2 << ' ' << f2 << ' ' << r2 << std::endl;
			std::cout << r3 << ' ' << b1 << ' ' << f1 << ' ' << r4 << std::endl;

		}

		void fpCoordinates()
		{
			float centre[2];
			centre[0] = len_x/2 + offset_x;
			centre[1] = len_y/2 + offset_y;

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

			set_radial_area(centre);

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

		bool radius_clearing(int index, float centre[2])
		{
			int coor[2];
			if (index <= len_x)
			{
				coor[0] = index;
			}
			else
			{
				coor[0] = (int)(index/len_y);
				coor[1] = (int)(index/len_x);
			}
			double dist = sqrt(pow(coor[0]-centre[0],2) + pow(coor[1]-centre[1], 2));
			return (dist > clear_radius);
		}

		void set_radial_area(float c[2])
		{
			for (int i=0; i <= len_x*len_y; i++)
			{
				if (radius_clearing(i, c) == false)
				{
					radial_area.push_back(i);
				}
			}
		}

		void checkclear(std::vector<signed char> cmap)
		{	

			bool left_clear=true, right_clear=true, up_clear=true, radius_clear=true, back_clear=true;
			/////////////////////////////////////////////
			int l=0, lf=0, f=0, rf=0, r=0, rb=0, b=0, lb=0, u=0, o=0;
			
			// left front box
			for (int i = f3; i <= f4; i+=len_x)
			{
				for (int j = i; j <= i + buffer; j++)
				{
					if (cmap[j] > 0)
					{	
						lf++;
						if (lf>=clear_tolerance)
						{
							left_clear = false;
							up_clear = false;
							goto endleft;
						}
					}
				}
			}
			endleftfront:

			for (int i = f1; i <= f2; i+=len_x)
			{
				for (int j = i; j <= i + buffer; j++)
				{
					if (cmap[j] > 0)
					{	
						rf++;
						if (rf>=clear_tolerance)
						{
							right_clear = false;
							up_clear = false;
							goto endright;
						}
					}
				}
			}
			endrightfront:

			for (int i = r3; i <= r1; i+=len_x)
			{
				for (int j = i; j <= i + buffer; j++)
				{
					if (cmap[j] > 0)
					{	
						rb++;
						if (rb>=clear_tolerance)
						{
							right_clear = false;
							back_clear = false;
							goto endrightback;
						}
					}
				}
			}
			endrightback:

			for (int i = l1; i <= l3; i+=len_x)
			{
				for (int j = i; j <= i + buffer; j++)
				{
					if (cmap[j] > 0)
					{	
						lb++;
						if (lb>=clear_tolerance)
						{
							left_clear = false;
							back_clear = false;
							goto endrightback;
						}
					}
				}
			}
			endleftback:
			
			/////////////////////////////////////////////
			// up clear
			u = rf + lf;
			for (int i = f2; i <= f3; i+=len_x)
			{
				for (int j = i; j <= i + (r2 - f2); j++)
				{
					if (cmap[j] > 0)
					{
						u+=1;
						if (u>=clear_tolerance)
						{	
							up_clear=false;
							goto endup;
						}
					}
				}
			}
			endup:

			// left clear
			l = lf+lb;
			for (int i = b3; i <= b4; i+=len_x)
			{
				for (int j = i; j <= i + (f3 - b3); j++)
				{
					if (cmap[j] > 0)
					{
						//left_clear = false;
						l += 1;
						if (l>=clear_tolerance)
						{	
							left_clear=false;
							goto endleft;
						}
						//break;
					}
				}
			}
			endleft:

			// right clear
			r = rf + rb;
			for (int i = b1; i <= b2; i+=len_x)
			{
				for (int j = i; j <= i + (f1 - b1); j++)
				{
					if (cmap[j] > 0)
					{
						r+=1;
						if (r>=clear_tolerance)
						{	
							right_clear=false;
							goto endright;
						}
					}
				}
			}
			endright:

			// back clear
			b = lb + rb;
			for ( int i = r1; i <= l1; i+=len_x)
			{
				for (int j = i; j <= i + (b2 - r1); j++)
				{
					if (cmap[i] > 0)
					{
						b++;
						if (b>=clear_tolerance)
						{
							back_clear = false;
							goto endback;
						}
					}
				}
			}
			endback:

			// radius clear
			for (int i : radial_area)
			{
				if (cmap[i] > 0)
				{	
					o++;
					if (o>=clear_tolerance)
					{
						radius_clear = false;
						goto endradius;
					}
				}
			}
			endradius:


			
			//std::cout << l << " " << r << " " << u << std::endl;
			auto* bl = &bools;
			bl->right = right_clear;
			bl->left = left_clear;
			bl->up = up_clear;
			bl->back = back_clear;
			bl->radius = radius_clear;
			cmap_clear.publish(*bl);
			//std::cout << *bl << std::endl;
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "costmap_clear_node");
	ros::NodeHandle nh;

	Robot Panthera = Robot(&nh);
	ros::spin();
	return 0;
}