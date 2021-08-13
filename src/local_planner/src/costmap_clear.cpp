/** TO DO LIST **/
// ADD SUBSCRIBER FOR ROBOT WIDTH
// CHECK fpCoordinates()

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
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

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
		ros::Subscriber RobotWidth;

		ros::Publisher cmap_clear;
		ros::Publisher robot_footprint;
		ros::Publisher search_area_pub;

		// Footprint info
		double length, width;
		std::array<int,2> left_back, left_front, right_back, right_front, centre;
		geometry_msgs::PolygonStamped fp, sa;

		float safety_dist;
		int buffer; // number of squares safety distance
		int clear_tolerance;
		double clear_radius;

		// search area
		int l1,l2,l3,l4,r1,r2,r3,r4,f1,f2,f3,f4,b1,b2,b3,b4;
		std::vector<int> radial_area;

		// Map info
		int len_x=10, len_y=10;
		double res=0.05;
		local_planner::CmapClear bools;

		// run once
		int n = 0;

	public:
		Robot(ros::NodeHandle *nh)
		{	
			CostMap = nh->subscribe("/semantics/costmap_generator/occupancy_grid", 10, &Robot::mapCallback, this);
			RobotWidth = nh->subscribe("/can_encoder", 10, &Robot::widthCallback, this);
			cmap_clear = nh->advertise<local_planner::CmapClear>("check_cmap",100);
			robot_footprint = nh->advertise<geometry_msgs::PolygonStamped>("/robot_footprint", 100);
			search_area_pub = nh->advertise<geometry_msgs::PolygonStamped>("/search", 100);

			length = nh->param("/robot_length", 2.2);
			width = nh->param("/robot_width", 1.0);
			safety_dist = nh->param("/safety_dist", 0.5);
			clear_tolerance = nh->param("/clear_tolerance", 1);
			clear_radius = nh->param("/clear_radius", 1.0);
		}

		void widthCallback(const geometry_msgs::Twist& msg)
		{	
			// robot width
			width = (msg.angular.y + msg.angular.z)/2 + 0.3; // +0.3 for length from wheel to cover, includes both sides
		}

		void mapCallback(const nav_msgs::OccupancyGrid& msg)
		{	
			len_x = msg.info.width;
			len_y = msg.info.height;
			res = msg.info.resolution;
			if (n == 0)
			{
				// init footprint points, does not consider reconfiguration
				fpCoordinates();
				n += 1;
			}
			std::vector<signed char> data_pts = msg.data; // occupancy grid data
			checkclear(data_pts); // check if area around robot is clear
			robot_footprint.publish(fp); // publishes robot footprint (PolygonStamped)
			search_area_pub.publish(sa); // publishes search area (PolygonStamped)
		}

		void searchArea()
		{	
			// Refer to top figure for corner names
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

			/**
			std::cout << l3 << ' ' << b4 << ' ' << f4 << ' ' << l4 << std::endl;
			std::cout << l1 << ' ' << b3 << ' ' << f3 << ' ' << l2 << std::endl;
			std::cout << r1 << ' ' << b2 << ' ' << f2 << ' ' << r2 << std::endl;
			std::cout << r3 << ' ' << b1 << ' ' << f1 << ' ' << r4 << std::endl;
			**/
			
		}

		void fpCoordinates()
		{
			float centre[2];
			centre[0] = len_x/2;
			centre[1] = len_y/2;

			float horz_dist = width/2; // y axis robot width
			float vert_dist = length/2; // x axis robot length

			int horz_pix = (int)ceil(horz_dist/res); // number of cells for robot width
			int vert_pix = (int)ceil(vert_dist/res); // number of cells for robot length
			buffer = (int)ceil(safety_dist/res);

			left_back[0] = (int)floor(centre[0] - vert_pix);
			left_back[1] = (int)ceil(centre[1] + horz_pix);

			left_front[0] = (int)ceil(centre[0] + vert_pix);
			left_front[1] = (int)ceil(centre[1] + horz_pix);

			right_back[0] = (int)floor(centre[0] - vert_pix);
			right_back[1] = (int)floor(centre[1] - horz_pix);

			right_front[0] = (int)ceil(centre[0] + vert_pix);
			right_front[1] = (int)ceil(centre[1] - horz_pix);

			// set search area for radius around robot
			set_radial_area(centre);

			searchArea();

			geometry_msgs::Point32 p1,p2,p3,p4;
			p1.y = -width/2;
			p1.x = -length/2;

			p2.y = -width/2;
			p2.x = length/2;

			p3.y = width/2;
			p3.x = length/2;

			p4.y = width/2;
			p4.x = -length/2;

			std::vector<geometry_msgs::Point32> footprint{p1, p2, p3, p4};
			fp.polygon.points = footprint;
			fp.header.frame_id = "footprint";
			//std::cout << fp.polygon.points[0] << std::endl;
			robot_footprint.publish(fp);

			search_area_viz(); // visualize search area in rviz
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

		void search_area_viz()
		{
			geometry_msgs::Point32 p1,p2,p3,p4;
			p1.y = -width/2 - safety_dist;
			p1.x = -length/2 - safety_dist;

			p2.y = -width/2 - safety_dist;
			p2.x = length/2 + safety_dist;

			p3.y = width/2 + safety_dist;
			p3.x = length/2 + safety_dist;

			p4.y = width/2 + safety_dist;
			p4.x = -length/2 - safety_dist;

			std::vector<geometry_msgs::Point32> search_area{p1, p2, p3, p4};
			sa.polygon.points = search_area;
			sa.header.frame_id = "search_area";
			search_area_pub.publish(sa);
		}

		bool radius_clearing(int index, float centre[2])
		{
			int coor[2];
			//clear_radius = 1.1/0.05;
			clear_radius = sqrt(pow(length/2,2) + pow(width/2,2))/res; // radius around robot during static rotation
			
			// convert index to x-y coordinates
			if (index <= len_x)
			{
				coor[0] = index;
				coor[1] = 0;
			}
			else
			{
				coor[0] = (int)(index/len_y);
				coor[1] = (int)(index/len_x);
			}
			double dist = sqrt(pow(coor[0]-centre[0],2) + pow(coor[1]-centre[1], 2));

			// check if point is within radius 
			return (dist > clear_radius);
		}

		void set_radial_area(float c[2])
		{
			for (int i=0; i <= len_x*len_y; i++)
			{	
				// add point to seach area if it is within robot radius
				if (radius_clearing(i, c) == false)
				{
					radial_area.push_back(i);
				}
			}

			// search area for visualization
			geometry_msgs::Point32 p1,p2,p3,p4;
			p1.y = -width/2 - safety_dist;
			p1.x = -length/2 - safety_dist;

			p2.y = -width/2 - safety_dist;
			p2.x = length/2 + safety_dist;

			p3.y = width/2 + safety_dist;
			p3.x = length/2 + safety_dist;

			p4.y = width/2 + safety_dist;
			p4.x = -length/2 - safety_dist;

			std::vector<geometry_msgs::Point32> search_area{p1, p2, p3, p4};
			sa.polygon.points = search_area;
			sa.header.frame_id = "search_area";
			search_area_pub.publish(sa);
		}

		void checkclear(const std::vector<signed char>& cmap)
		{	
			// check if areas are clear

			/**
			 * Right: right_front + right_back + right
			 * Left: left_front + left + left_back
			 * Up: left_front + up + right_up
			 * Back: left_back + back + right_back
			**/
			
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
						if (lf>=clear_tolerance) // if number of points are more than tolerance for noise, mark area as not clear
						{
							left_clear = false;
							up_clear = false;
							goto endleftfront;
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
							goto endrightfront;
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
							goto endleftback;
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
				for (int j = i; j <= i + buffer; j++)
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
			/**
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
			**/
			o = lb+l+lf+u+rf+r+rb+b;
			if (o >= clear_tolerance)
			{
				radius_clear = false;
			}

			//std::cout << l << " " << r << " " << u << std::endl;

			// publish checked areas
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