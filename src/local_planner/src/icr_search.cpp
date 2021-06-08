/** TO DO LIST **/
// ADD SUBSCRIBER FOR ROBOT WIDTH
// CHECK fpCoordinates()

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <local_planner/CmapClear.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <cmath>
#include <local_planner/astar.h>

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
		ros::Subscriber turn_angle;

		ros::Publisher cmap_clear;
		ros::Publisher robot_footprint;
		ros::Publisher search_area_pub;

		// Footprint info
		double length, width;
		std::vector<geometry_msgs::Point32> footprint_points;
		std::vector<geometry_msgs::Point32> possible_icr;

		float safety_dist;
		int buffer; // number of squares safety distance
		int clear_tolerance;
		double clear_radius;

		// search area
		//int l1,l2,l3,l4,r1,r2,r3,r4,f1,f2,f3,f4,b1,b2,b3,b4;
		//int b2, b3, f2, f3;

		geometry_msgs::Point32 lb, lf, rb, rf;
		std::vector<geometry_msgs::Point32> footprint{lb, lf, rb, rf};
		// std::vector<int> radial_area;

		// Map info
		int len_x=10, len_y=10;
		double res=0.05;
		local_planner::CmapClear bools;

		std::vector<signed char> data_pts;

		// run once
		int n = 0;

		// angle to rotate
		double angle;

	public:
		Robot(ros::NodeHandle *nh)
		{	
			CostMap = nh->subscribe("/semantics/costmap_generator/occupancy_grid", 10, &Robot::mapCallback, this);
			RobotWidth = nh->subscribe("/can_encoder", 10, &Robot::widthCallback, this);
			turn_angle = nh->subscribe("/turn_angle", 10, &Robot::rotation_angle, this); // subscribe to get angle to rotate
			//robot_footprint = nh->advertise<geometry_msgs::PolygonStamped>("/robot_footprint", 100);
			//search_area_pub = nh->advertise<geometry_msgs::PolygonStamped>("/search", 100);

			length = nh->param("/robot_length", 2.2);
			width = nh->param("/robot_width", 1.0);
			safety_dist = nh->param("/safety_dist", 0.5);
			clear_tolerance = nh->param("/clear_tolerance", 1);
			clear_radius = nh->param("/clear_radius", 1.0);
		}

		void rotation_angle(const std_msgs::Float64& theta)
		{
			angle = theta.data;
			std::cout << footprint_points.size() << std::endl;
			int count=1;
			for (auto i : footprint_points)
			{	

				if (rotation_clear(i, angle) == true)
				{
					possible_icr.push_back(i);
				}
				std::cout << count << std::endl;
				count++;
			}
			std::cout << possible_icr.size() << std::endl;
			for (auto i : possible_icr)
			{
				std::cout << i << std::endl;
			}
		}

		void widthCallback(const geometry_msgs::Twist& msg)
		{
			width = (msg.angular.y + msg.angular.z)/2 + 0.3;
		}

		void mapCallback(const nav_msgs::OccupancyGrid& msg)
		{	
			len_x = msg.info.width;
			len_y = msg.info.height;
			res = msg.info.resolution;
			if (n == 0)
			{
				cornerPoints();
				n += 1;
			}
			data_pts = msg.data;
		}

		void cornerPoints()
		{	
			lb.x = (len_x/2 - length/2)/res;
			lb.y = (len_y/2 + width/2)/res;

			lf.x = (len_x/2 + length/2)/res;
			lf.y = (len_y/2 + width/2)/res;

			rb.x = (len_x/2 - length/2)/res;
			rb.y = (len_y/2 - width/2)/res;

			rf.x = (len_x/2 + length/2)/res;
			rf.y = (len_y/2 + width/2)/res;

			// get robot cells
			for (int j=rb.y; j<= lb.y; j++)
			{
				for (int i=lb.x; i<=lf.x; i++)
				{
					geometry_msgs::Point32 pt;
					pt.x = i;
					pt.y = j;
					footprint_points.push_back(pt);
				}
			}
		}

		std::vector<geometry_msgs::Point32> outlinepolygon(std::vector<geometry_msgs::Point32> polygon) // inputs vector of point32 (map coordinates)
		{	
			std::vector<geometry_msgs::Point32> outline;
			for (int i=0; i < polygon.size()-1; i++)
			{	
				double AB = distance(polygon[i].x, polygon[i].y, polygon[i+1].x, polygon[i+1].y);
				
				int x0,x1,y0,y1;

				if (polygon[i].x > polygon[i+1].x)
				{
					x0 = polygon[i+1].x;
					x1 = polygon[i].x;
				}
				else
				{	
					x0 = polygon[i].x;
					x1 = polygon[i+1].x;
				}

				if (polygon[i].y > polygon[i+1].y)
				{
					y0 = polygon[i+1].y;
					y1 = polygon[i].y;
				}
				else
				{	
					y0 = polygon[i].y;
					y1 = polygon[i+1].y;
				}

				for (int x=x0; x<=x1; x++)
				{
					for (int y=y0; y<=y1; y++)
					{
						// distance 
						double AE = distance(polygon[i].x, polygon[i].y, x, y);
						double BE = distance(polygon[i+1].x, polygon[i+1].y, x, y);
						double theta = acos(pow(AB,2) + pow(BE,2) - pow(AE,2)/(2*AB*BE));
						double distance = BE * sin(theta);

						if (distance < res)
						{
							geometry_msgs::Point32 pt;
							pt.x = x;
							pt.y = y;

							if (std::count(outline.begin(), outline.end(), pt)){}
							else
							{
								outline.push_back(pt);
							}
						}
					}
				}
			}
			std::sort(outline.begin(), outline.end(), sort_y);
			return outline;
		}

		double distance(int x0, int y0, int x1, int y1)
		{
			return sqrt(pow(x0 - x1,2) + pow(y0 - y1,2));
		}

		std::vector<int> fill_polygon_outline(std::vector<geometry_msgs::Point32> outline) // sorted vector according to y value
		{	
			printf("Filling polygon outline...\n");
			std::vector<int> filled_cells;
			for (int i=0; i<(int)(outline.size())-1; i++)
			{
				int current = coordinates_to_index((outline[i]).x, (outline[i]).y, len_x/res);
				filled_cells.push_back(current);
				if ((outline[i]).y == (outline[i+1]).y)
				{
					int c = (int)(outline[i].x);
					while (c != (int)(outline[i+1].x))
					{
						filled_cells.push_back(coordinates_to_index(c, outline[i].y, len_x/res));
						c++;
					}
				}
			}
			printf("Filled polygon outline\n");
			return filled_cells;
		}

		static bool sort_y(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b)
		{
			return (a.y < b.y);
		}

		std::vector<geometry_msgs::Point32> sector_footprint(geometry_msgs::Point32 icr, geometry_msgs::Point32 corner, double theta) // angle in radians
		{
			std::vector<geometry_msgs::Point32> footprint{icr};
			int interval = 5; // number of points on arc
			double radius = distance(icr.x, icr.y, corner.x, corner.y);
			double angle_interval = std::ceil(theta/interval);

			for (int i=0; i<=interval; i++)
			{
				footprint.push_back(rotate_pt(corner, icr, i*angle_interval));
			}
			return footprint; // outline
		}

		geometry_msgs::Point32 rotate_pt(geometry_msgs::Point32 pt, geometry_msgs::Point32 icr, double gamma)
		{
			geometry_msgs::Point32 tf_corner;
			tf_corner.x = pt.x - icr.x;
			tf_corner.y = pt.y - icr.y;
			geometry_msgs::Point32 new_corner;
			new_corner.x = tf_corner.x*sin(gamma) - tf_corner.y*cos(gamma);
			new_corner.y = tf_corner.x*sin(gamma) + tf_corner.y*cos(gamma);

			new_corner.x += icr.x;
			new_corner.y += icr.y;

			new_corner.x = std::round(new_corner.x);
			new_corner.y = std::round(new_corner.y);

			return new_corner;
		}

		std::vector<geometry_msgs::Point32> new_fp(geometry_msgs::Point32 icr, double theta)
		{	

			std::vector<geometry_msgs::Point32> fp;
			for (int i=0; i<(int)(footprint.size()); i++)
			{
				fp.push_back(rotate_pt(footprint[i], icr, theta));
			}
			return fp;
		}

		std::vector<int> concat_search_area(std::vector<int> fp, std::vector<int> c1, std::vector<int> c2, std::vector<int> c3, std::vector<int> c4)
		{	
			printf("Concating search area...\n");
			std::vector<int> search_area;
			search_area.reserve(fp.size() + c1.size() + c2.size() + c3.size() + c4.size());
			search_area.insert(search_area.end(), fp.begin(), fp.end());
			search_area.insert(search_area.end(), c1.begin(), c1.end());
			search_area.insert(search_area.end(), c2.begin(), c2.end());
			search_area.insert(search_area.end(), c3.begin(), c3.end());
			search_area.insert(search_area.end(), c4.begin(), c4.end());
			printf("Got search area\n");
			return search_area;
		}

		bool rotation_clear(geometry_msgs::Point32 icr, double theta)
		{
			std::vector<int> search_area;

			// get new footprint
			std::vector<geometry_msgs::Point32> rotated_fp = new_fp(icr, theta);
			std::vector<int> filled_rotated_fp = fill_polygon_outline(outlinepolygon(rotated_fp));

			// get sector cells
			std::vector<std::vector<int>> sectors(4);
			for (int i=0; i<footprint.size(); i++)
			{
				sectors[i] = fill_polygon_outline(sector_footprint(footprint[i], icr, theta));
			}
			std::vector<int> sa = concat_search_area(filled_rotated_fp, sectors[0], sectors[1], sectors[2], sectors[3]);

			for (auto i : sa)
			{
				if (data_pts[i] > 0)
				{
					return false;
				}
			}
			return true;
		}
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icr_search_node");
	ros::NodeHandle nh;

	Robot Panthera = Robot(&nh);
	ros::spin();
	return 0;
}