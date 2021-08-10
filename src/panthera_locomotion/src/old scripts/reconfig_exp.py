#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusRequest, StatusResponse
from ds4_driver.msg import Status as st
import math

def ds4_sub(msg):
	forward = msg.button_dpad_up
	reverse = -msg.button_dpad_down
	v = -msg.button_circle # face outwards
	inv_v = msg.button_square # face inwards
	open_close = v + inv_v
	direction = msg.axis_left_y

	l2 = msg.button_l2
	r2 = msg.button_r2
	x = msg.button_cross

	reconfig(direction, reconfig_speed, open_close)
	#half_reconfig(l2, r2, direction, x)

def reconfig(direction, speed, oc):
	twist = Twist()
	twist.linear.x = -20 * oc
	twist.linear.y = 20 * oc 
	twist.linear.z = -20 * oc
	twist.angular.x = 20 * oc
	#twist.angular.y = 0
	twist.angular.z = 0

	#check()

	twist.angular.y = reconfig_speed*direction
	cmd_pub.publish(twist)

def half_reconfig(l2, r2, speed, sign):
	twist = Twist()
	angles = Twist()
	angles.linear.x = l2*20
	angles.linear.y = r2*-20
	angles.linear.z = l2*20
	angles.angular.x = r2*-20

	cmd_pub.publish(angles)

	twist.linear.x = l2*speed/math.cos(20) + r2*speed
	twist.linear.y = r2*speed/math.cos(20) + l2*speed
	twist.linear.z = l2*speed/math.cos(20) + r2*speed
	twist.angular.x = r2*speed/math.cos(20) + l2*speed

	wheel_speed.publish(twist)

def check():
	req = StatusRequest()
	req.reconfig = True
	signal = False
	rate = rospy.Rate(2)
	while signal == False and not rospy.is_shutdown():
		rate.sleep()
		lb = lb_status(req)
		rb = rb_status(req)
		lf = lf_status(req)
		rf = rf_status(req)
		signal = (lb.status and rb.status and lf.status and rf.status)
		print([lb.status, rb.status, lf.status, rf.status])
		print("Status of steering motors:" + str(signal))

if __name__ == "__main__":
	rospy.init_node("controller")
	reconfig_speed = 0.08
	rospy.Subscriber("/status", st, ds4_sub)
	mode = 0
	cmd_pub = rospy.Publisher("/panthera_cmd", Twist, queue_size=1)
	wheel_speed = rospy.Publisher("/reconfig", Twist, queue_size=1)
	lb_status = rospy.ServiceProxy('lb_steer_status', Status)
	lf_status = rospy.ServiceProxy('lf_steer_status', Status)
	rb_status = rospy.ServiceProxy('rb_steer_status', Status)
	rf_status = rospy.ServiceProxy('rf_steer_status', Status)
	rospy.spin()