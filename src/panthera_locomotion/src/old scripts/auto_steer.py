#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist

width = 0.86
length = 1.31

lf_current = 0
lb_current = 0
rf_current = 0
rb_current = 0

lf_target = 0
lb_target = 0
rf_target = 0
rb_target = 0

def encoder_sub(data):
	global lf_current, lb_current, rb_current, rf_current
	lb_current = data.linear.x
	rb_current = data.linear.y
	lf_current = data.linear.z
	rf_current = data.angular.x

def adjust_wheels(vx, wz):
	global lb_target, rb_target, lf_target, rf_target
	if wz == 0:
		radius = float('inf')
	else:
		radius = vx/wz
	left = radius - width/2
	right = radius + width/2
	
	lf_target = round(math.degrees(math.atan((length*0.5) / left)), 2)
	rf_target = round(math.degrees(math.atan((length*0.5) / right)), 2)
	lb_target = -lf_target
	rb_target = -rf_target

def adjust_steer_speed():
	d_lb = int(abs(lb_current - lb_target))
	d_rb = int(abs(rb_current - rb_target))
	d_lf = int(abs(lf_current - lf_target))
	d_rf = int(abs(rf_current - rf_target))

	if d_rf > d_lf:
		rf_speed = d_rf/d_lf
		return 1, 1, 1, rf_speed
	else:
		lf_speed = d_lf/d_rf
		return 1, lf_speed, 1, 1

def run():
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		

if __name__ == "__main__":
	try:
		rospy.init_node('steering_node')
		rospy.Subscriber('/encoder_positions', Twist, encoder_sub)
		target_pub = rospy.Publisher('/target_angle', Twist, queue_size=1)
		steer_speed_pub = rospy.Publisher('/steer_speed', Twist, queue_size=1)



