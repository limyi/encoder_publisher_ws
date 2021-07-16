#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusRequest, StatusResponse
from ds4_driver.msg import Status as st
import math

def ds4_sub(msg):
	forward = msg.button_dpad_up
	reverse = -msg.button_dpad_down
	direction = forward + reverse
	reconfig(direction, reconfig_speed)

def reconfig(direction, speed):
	twist = Twist()
	twist.linear.x = -45*abs(direction)
	twist.linear.y = 45*abs(direction)
	twist.linear.z = -45*abs(direction)
	twist.angular.x = 45*abs(direction)
	twist.angular.y = 0
	twist.angular.z = 0

	#check()

	twist.angular.y = direction*2*speed/math.sqrt(2)
	cmd_pub.publish(twist)

def check(self):
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
	reconfig_speed = 0.2
	rospy.Subscriber("/status", st, ds4_sub)
	cmd_pub = rospy.Publisher("/panthera_cmd", Twist, queue_size=1)
	lb_status = rospy.ServiceProxy('lb_steer_status', Status)
	lf_status = rospy.ServiceProxy('lf_steer_status', Status)
	rb_status = rospy.ServiceProxy('rb_steer_status', Status)
	rf_status = rospy.ServiceProxy('rf_steer_status', Status)
	rospy.spin()