#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

class PoseRecorder
{

private :
	ros::Subscriber pose_sub;
	ros::Publisher path_pub;

	nav_msgs::Path robot_path;

public :
	PoseRecorder(ros::NodeHandle *nh)
	{
		pose_sub = nh->subscribe("/ndt_pose", 100, &PoseRecorder::PoseSub, this);
		path_pub = nh->advertise<nav_msgs::Path>("/recorded_path", 100);
	}

	void PoseSub(const geometry_msgs::PoseStamped& msg)
	{	
		auto* new_path = &robot_path;
		ros::Rate rate(1);
		new_path->poses.push_back(msg);
		path_pub.publish(robot_path);
		rate.sleep();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_recorder_node");
	ros::NodeHandle nh;
	PoseRecorder pr = PoseRecorder(&nh);
	ros::spin();
	return 0;
}