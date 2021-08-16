#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status
import math

vx = 0
compress = 0
expand = 0

left = 0
right = 0
centre = 0

wheel_vel = Twist()
wheel_angle = Twist()

def vx_callback(msg):
	global vx
	vx = msg.linear.x * 0.1

def ds4_buttons(msg):
	global compress, expand, left, right, centre, wheel_angle, wheel_vel
	compress = msg.button_r2
	expand = msg.button_l2

	left = msg.button_l1
	right = msg.button_r1
	centre = msg.button_cross

	wheel_angle.linear.x = (left + centre) * (-compress + expand) * 15#7.6
	wheel_angle.linear.y = (right + centre) * (compress - expand) * 15#7.6
	wheel_angle.linear.z = (left + centre) * (-compress + expand) * 15#7.6
	wheel_angle.angular.x = (right + centre) * (compress - expand) * 15#7.6
	pub.publish(wheel_angle)

	wheel_vel.linear.x = vx / abs(math.cos(wheel_angle.linear.x))
	wheel_vel.linear.y = vx / abs(math.cos(wheel_angle.linear.y))
	wheel_vel.linear.z = vx / abs(math.cos(wheel_angle.linear.z))
	wheel_vel.angular.x = vx / abs(math.cos(wheel_angle.angular.x))
	speed.publish(wheel_vel)

if __name__ == '__main__':
	rospy.init_node("madan_exp")
	rospy.Subscriber("/cmd_vel", Twist, vx_callback)
	rospy.Subscriber("/status", Status, ds4_buttons)
	pub = rospy.Publisher("/panthera_cmd", Twist, queue_size=1)
	speed = rospy.Publisher("/reconfig", Twist, queue_size=1)
	print("Expand: L2")
	print("Compress: R2")
	print("Left: L1")
	print("Right: R1")
	print("Centre: X")
	rospy.spin()