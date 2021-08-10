#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PolygonStamped, Point32
from geometry_msgs.msg import PoseStamped, Twist
import tf



def footprint_points(msg):
	fp = PolygonStamped()

	lb = Point32()
	lb.x = -robot_length/2
	lb.y = (msg.angular.y)/2 + 0.15

	lf = Point32()
	lf.x = robot_length/2
	lf.y = (msg.angular.y)/2 + 0.15

	rf = Point32()
	rf.x = robot_length/2
	rf.y = -(msg.angular.y)/2 - 0.15

	rb = Point32()
	rb.x = -robot_length/2
	rb.y = -msg.angular.y/2 - 0.15

	fp.polygon.points = [rb,lb,lf,rf]
	fp.header.frame_id = "velodyne"
	pub.publish(fp)
	print(fp)

if __name__ == "__main__":
	rospy.init_node("fp_pub")
	robot_width = 1.0
	robot_length = 1.7
	pub = rospy.Publisher("/footprint", PolygonStamped, queue_size=100)
	#rospy.Subscriber("ndt_pose", PoseStamped, tf_sub)
	rospy.Subscriber("/can_encoder", Twist, footprint_points)
	rospy.spin()