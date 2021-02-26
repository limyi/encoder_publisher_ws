#!/usr/bin/env python

import rospy
import math
import orienbus
from geometry_msgs.msg import Twist
import serial.tools.list_ports
from std_msgs.msg import Float32

############## Change for different motor ###############
address = 1
p = list(serial.tools.list_ports.grep(rospy.get_param('/rf_sn')))
port = '/dev/' + p[0].name
#########################################################

orienbus = orienbus.OrienBus(port)
motor = orienbus.initialize(address)

width = 0.89
length = 1.31
wheel_radius = 0.1

#run_mode = rospy.get_param('/run_mode')

####### Check if gear ratio correct #######
gear_ratio = 100

def reconfig(data): ###
	global wheel_speed
	wheel_speed = -rads_to_rpm(data.angular.x / wheel_radius)

def callback(data):
	global linear_x
	global angular_z
	linear_x = data.angular.y
	angular_z = data.angular.z

def motor_lin_vel(vx, wz):
	sign = wz / abs(wz)
	r = abs(vx / wz)
	speed = math.sqrt((r - sign*width/2)**2 + (length/2)**2) * abs(wz) # check + or - 
	if r < width/2 and vx/wz < 0:
		speed  = -speed
	else:
		pass
	return speed # check motor direction 

def rads_to_rpm(x):
	rpm = (x / (2*math.pi)) * 60 * gear_ratio
	return int(-rpm)

def rpm_to_rads(x):
	rads = (x/60) * 2 * math.pi / gear_ratio
	return (rads)

def adjust_speed(vx, wz):
	speed = 0
	#if run_mode == True:
	if vx == 0:
		if wz == 0:
			rpm = 0
			motor.writeSpeed(rpm)
			#print("rf rpm: 0")

		else:
			speed = wz * math.sqrt((length/2)**2 + (width/2)**2) / wheel_radius
			rpm = rads_to_rpm(speed)
			#print("rf rpm: " + str(rpm))
			motor.writeSpeed(rpm)			

	else:
		if wz == 0:
			speed = vx / wheel_radius
			rpm = rads_to_rpm(speed)
			#print("rf rpm: " + str(rpm))
			motor.writeSpeed(rpm)

		else:
			lin_vel  = motor_lin_vel(vx, wz)
			speed = lin_vel / wheel_radius

			rpm = rads_to_rpm(speed)
			#print("rf rpm: " + str(rpm))
			motor.writeSpeed(rpm)
	#else:
		#print("rf rpm: " + str(rpm))

if __name__ == "__main__":
	try:
		linear_x = 0
		angular_z = 0
		wheel_speed = 0

		########## Change node name #############
		rospy.init_node('rf_trans_motor')
		
		sub = rospy.Subscriber('target_angle', Twist, callback)

		sub2 = rospy.Subscriber('reconfig', Twist, reconfig) # Twist message for motor speed during reconfiguration
		rate = rospy.Rate(20)
		pub = rospy.Publisher('/rf_wheel_vel', Float32, queue_size=1)
		while not rospy.is_shutdown():
			if wheel_speed == 0:
				adjust_speed(linear_x, angular_z)

			else:
				#print("rf rpm: " + str(wheel_speed))
				motor.writeSpeed(wheel_speed)
			speed = rpm_to_rads(motor.readSpeed()) * wheel_radius
			#print(speed)
			pub.publish(speed)
			rate.sleep()

	except rospy.ROSInterruptException:
		pass





