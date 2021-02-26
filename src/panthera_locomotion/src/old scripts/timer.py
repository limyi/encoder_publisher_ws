#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

def x(event):
	print(event.current_real)

def callback():
	print("Start")
	rospy.sleep(2)
	print("End")


if __name__ == "__main__":
	try:
		rospy.init_node('test_loop')
		#pub = rospy.Publisher('target_angle', Twist, queue_size=1)
		rate = rospy.Rate(0.5)
		while not rospy.is_shutdown():
			callback()
			rate.sleep()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass