#!/usr/bin/env python

import rospy
from autoware_msgs.msg import LaneArray
import math

def callback(msg):
	for i in range(len(msg.lanes[0].waypoints)-5):
		diff = math.sqrt((msg.lanes[0].waypoints[i].pose.pose.position.x - msg.lanes[0].waypoints[i+1].pose.pose.position.x)**2 + (msg.lanes[0].waypoints[i].pose.pose.position.y - msg.lanes[0].waypoints[i+1].pose.pose.position.y)**2)
		print(diff)


if __name__ == "__main__":
	rospy.init_node("lane_sub")
	rospy.Subscriber("lane_waypoints_array", LaneArray, callback)
	rospy.spin()