#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Twist, Point32

def width_callback(msg):
	# adjust footprint width to robot width
	width = msg.angular.z + 0.3

	ps = PolygonStamped()
	ps.header.frame_id = "velodyne" # fixed robot footprint to velodyne

	# corners of footprint
	lb = Point32()
	lf = Point32()
	rf = Point32()
	rb = Point32()

	lb.x = -length/2
	lb.y = width/2

	lf.x = length/2
	lf.y = width/2

	rf.x = length/2
	rf.y = -width/2

	rb.x = -length/2
	rb.y = -width/2

	ps.polygon.points = [lb,lf,rf,rb]
	footprint_pub.publish(ps)

if __name__=="__main__":
	rospy.init_node("robot_footprint")
	length = 2.2
	width = 0.7
	rospy.Subscriber("/can_encoder", Twist, width_callback)
	footprint_pub = rospy.Publisher("/footprint", PolygonStamped, queue_size=1)
	rospy.spin()