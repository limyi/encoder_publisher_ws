#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusRequest, StatusResponse

radius = 0.5

lb_speed = 0
rb_speed = 0
lf_speed = 0
rf_speed = 0

lb_trans = 0
rb_trans = 0
lf_trans = 0
rf_trans = 0

# OLD CHECK
def reset_check():
	global lb_speed, rb_speed, lf_speed, rf_speed
	req = StatusRequest()
	signal = False
	rate = rospy.Rate(2)
	while signal  == False and not rospy.is_shutdown():
		rate.sleep()
		lb = lb_status(req)
		rb = rb_status(req)
		lf = lf_status(req)
		rf = rf_status(req)
		signal = (lb.status and rb.status and lf.status and rf.status)
		print([lb.status, rb.status, lf.status, rf.status])
		print("Status of steering motors:" + str(signal))

def moving_check(sign=1):
	global lb_speed, rb_speed, lf_speed, rf_speed
	twist = Twist()
	req = StatusRequest()
	signal = False
	rate = rospy.Rate(2)
	while signal  == False and not rospy.is_shutdown():
		rate.sleep()
		lb = lb_status(req)
		rb = rb_status(req)
		lf = lf_status(req)
		rf = rf_status(req)
		lb_speed = lb.speed
		rb_speed = rb.speed
		lf_speed = lf.speed
		rf_speed = rf.speed
		twist.linear.x = sign*adjust_trans(lb_speed, "lb")
		twist.linear.z = sign*adjust_trans(lb_speed, "rb")
		twist.linear.y = sign*adjust_trans(lb_speed, "lf")
		twist.angular.x = sign*adjust_trans(lb_speed, "rf")
		pub2.publish(twist)
		signal = (lb.status and rb.status and lf.status and rf.status)
		print([lb.status, rb.status, lf.status, rf.status])
		print("Status of steering motors:" + str(signal))

def adjust_trans(speed, wheel):
	vel = 0
	if abs(speed) < 80:
		vel = 0
	else:
		if wheel == "lb":
			w = -speed/60*2*math.pi/200
			vel = w*radius
		else:
			w = -speed/60*2*math.pi/200
			vel = w*radius
	print(wheel, vel)
	return vel

def countdown():
	ts = Twist()
	t2 = Twist()

	####### Set default to 90 deg
	ts.linear.x = 90
	ts.linear.z = 90
	ts.linear.y = 90
	ts.angular.x = 90
	pub.publish(ts)
	
	####### Wait for wheels to be 90deg
	print("Setting Default to 90 deg")
	rospy.sleep(1)
	reset_check()

	
	rate = rospy.Rate(1)
	
	####### Start movement to -90 deg
	ts.linear.x = -90
	ts.linear.z = -90
	ts.linear.y = -90
	ts.angular.x = -90
	pub.publish(ts)
	'''
	trig = input("Start? [y/n]: ")
	print(trig, type(trig))
	'''
	
	print("Moving to -90 deg")
	moving_check()

	####### Stop trans motors, reset wheels to 90 deg
	t2.linear.x = 0
	t2.linear.z = 0
	t2.linear.y = 0
	t2.angular.x = 0
	pub2.publish(t2)

	ts.linear.x = 90
	ts.linear.z = 90
	ts.linear.y = 90
	ts.angular.x = 90
	pub.publish(ts)

	######## Wait for wheels to be 90 deg
	print("Resetting to 90 deg")
	reset_check()

	rospy.sleep(1)
	####### Set -90 deg target
	ts.linear.x = -90
	ts.linear.z = -90
	ts.linear.y = -90
	ts.angular.x = -90
	
	####### Start moving to -90 deg
	print("Moving to -90 deg")
	moving_check(-1)

	###### Stop trans
	t2.linear.x = 0
	t2.linear.z = 0
	t2.linear.y = 0
	t2.angular.x = 0
	pub2.publish(t2)

	'''
	now = rospy.get_time()
	later = rospy.get_time()
	while later - now < 10:
		pub.publish(ts)
		check()
		later = rospy.get_time()
	'''
	'''
	rate = rospy.Rate(0.5)
	ts.linear.x = -90
	ts.linear.z = -90
	ts.linear.y = -90
	ts.angular.x = -90
	pub.publish(ts)
	t2.linear.x = 0.5
	t2.linear.z = 0.5
	t2.linear.y = 0.5
	t2.angular.x = 0.5
	pub2.publish(t2)

	check()

	t2.linear.x = 0
	t2.linear.z = 0
	t2.linear.y = 0
	t2.angular.x = 0
	pub2.publish(t2)
	'''
	################################################3
	#check()
	#print("Checking")
	'''
	rate = rospy.Rate(0.3)
	count = 90
	while count >= -90 and not rospy.is_shutdown():
		ts.linear.x = count
		ts.linear.z = count
		ts.linear.y = count
		ts.angular.x = count
		t2.linear.x = -0.1
		t2.linear.z = -0.1
		t2.linear.y = -0.1
		t2.angular.x = -0.1
		pub.publish(ts)
		pub2.publish(t2)
		count-=10
		print("Angle: {}".format(count+10))
		rate.sleep()

	now = rospy.get_time()
	later = rospy.get_time()
	while later - now < 8:
		pub.publish(ts)
		check()
		later = rospy.get_time()

	t2.linear.x = 0
	t2.linear.z = 0
	t2.linear.y = 0
	t2.angular.x = 0
	pub2.publish(t2)
	ts.linear.x = 90
	ts.linear.z = 90
	ts.linear.y = 90
	ts.angular.x = 90
	pub.publish(ts)

	now = rospy.get_time()
	later = rospy.get_time()
	while later - now < 10:
		pub.publish(ts)
		check()
		later = rospy.get_time()
	#print("Checking")

	count = 90
	while count >= -90 and not rospy.is_shutdown():
		ts.linear.x = count
		ts.linear.z = count
		ts.linear.y = count
		ts.angular.x = count
		t2.linear.x = 0.1
		t2.linear.z = 0.1
		t2.linear.y = 0.1
		t2.angular.x = 0.1
		pub.publish(ts)
		pub2.publish(t2)
		count-=10
		print("Angle: {}".format(count+10))
		rate.sleep()
	'''
def check():
	req = StatusRequest()
	signal = False
	rate = rospy.Rate(2)
	while signal  == False and not rospy.is_shutdown():
		rate.sleep()
		lb = lb_status(req)
		rb = rb_status(req)
		lf = lf_status(req)
		rf = rf_status(req)
		signal = (lb.status and rb.status and lf.status and rf.status)
		print([lb.status, rb.status, lf.status, rf.status])
		print("Status of steering motors:" + str(signal))

def reset():
	twist = Twist()
	twist.linear.x = 90
	twist.linear.y = 90
	twist.linear.z = 90
	twist.angular.x = 90
	pub.publish(twist)
	print(twist)
	
	now = rospy.get_time()
	later = rospy.get_time()
	while later - now < 2:
		later = rospy.get_time()
	check()
	
def left_half():
	reset()
	ts = Twist()
	t2 = Twist()
	reset_check()
	ts.linear.x = -90
	ts.linear.y = -90
	ts.linear.z = -90
	ts.angular.x = -90
	pub.publish(ts)
	moving_check(1)
	t2.linear.x = 0
	t2.linear.y = 0
	t2.linear.z = 0
	t2.angular.x = 0
	pub2.publish(t2)
	reset()

def right_half():
	reset()
	ts = Twist()
	t2 = Twist()
	ts.linear.x = -90
	ts.linear.y = -90
	ts.linear.z = -90
	ts.angular.x = -90
	pub.publish(ts)
	moving_check(-1)
	t2.linear.x = 0
	t2.linear.y = 0
	t2.linear.z = 0
	t2.angular.x = 0
	pub2.publish(t2)
	reset()




if __name__ == "__main__":
	try:
		rospy.init_node("counter")
		pub = rospy.Publisher("target_angle", Twist, queue_size=1)
		pub2 = rospy.Publisher("reconfig", Twist, queue_size=1)
		
		rospy.wait_for_service('/lb_steer_status')
		rospy.wait_for_service('/lf_steer_status')
		rospy.wait_for_service('/rb_steer_status')
		rospy.wait_for_service('/rf_steer_status')

		lb_status = rospy.ServiceProxy('lb_steer_status', Status)
		lf_status = rospy.ServiceProxy('lf_steer_status', Status)
		rb_status = rospy.ServiceProxy('rb_steer_status', Status)
		rf_status = rospy.ServiceProxy('rf_steer_status', Status)
		
		reset()
		count = 0
		while count < 1:
			left_half()
			right_half()
			count += 1
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
