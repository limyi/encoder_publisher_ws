#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

def callback():
	twist = Twist()
	angle = input("Enter angle: ")
	twist.linear.x = angle
	twist.linear.y = angle
	twist.linear.z = angle
	twist.angular.x = angle
	pub.publish(twist)


if __name__ == "__main__":
	try:
		rospy.init_node('test_loop')
		pub = rospy.Publisher('target_angle', Twist, queue_size=1)
		while not rospy.is_shutdown():
			callback()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass