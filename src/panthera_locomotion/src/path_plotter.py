#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

path = []
robot_path = Path()

def path_sub(msg):
	path.append(msg)
	robot_path.poses = path
	robot_path.header.frame_id = "map"
	pub.publish(robot_path)

if __name__ == "__main__":
	rospy.init_node("path_plotter")
	rospy.Subscriber("/ndt_pose", PoseStamped, path_sub)
	pub = rospy.Publisher("/pose_path", Path, queue_size=1)
	rospy.spin()