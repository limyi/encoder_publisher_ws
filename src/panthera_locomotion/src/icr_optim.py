#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid as og
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def pub_width():
	twist = Twist()
	twist.angular.y = 1.0
	twist.angular.z = 1.0
	can.publish(twist)

def pub_map(res, width, height):
	grid = og()
	grid.info.resolution = res
	grid.info.width = width
	grid.info.height = height
	grid.header.frame_id = "costmap"
	data = []
	for i in range(50):
		data.append(0)
	grid.data = data
	cmap.publish(grid)

if __name__ == "__main__":
	rospy.init_node("pubs")
	can = rospy.Publisher("/can_encoder", Twist, queue_size=1)
	cmap = rospy.Publisher("/semantics/costmap_generator/occupancy_grid",og, queue_size=1)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		pub_width()
		pub_map(0.1, 5, 5)
		rate.sleep()



